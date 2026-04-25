import os
import time
import glob
import platform
import threading

import cv2
import serial
import serial.tools.list_ports
import torch
from ultralytics import YOLO


# ---------- Config ----------
MODEL_PATH = "best_03_14.pt"
TARGET_CLASS = "blue_cube"
BAUD = 115200

CAMERA_INDEX = 0
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
INFER_IMGSZ = 512
CONF_THRESH = 0.45

# Servo limits / homes (must match the .ino)
BASE_MIN, BASE_MAX, BASE_HOME = 0, 90, 45
CAM_MIN,  CAM_MAX,  CAM_HOME  = 35, 120, 40

# Tracking (P-only, fixed-rate)
KP             = 0.004
DEADZONE_PX    = 35
MAX_STEP_DEG   = 2
EMA_ALPHA      = 0.35
CMD_GAP_SEC    = 0.10
BASE_CMD_GAP_SEC = CMD_GAP_SEC / 1.3
ALIGN_HOLD_SEC = 0.55

PREPICK_TIMEOUT_SEC = 8.0
GRAB_TIMEOUT_SEC    = 12.0


# ---------- Camera thread ----------
class CameraThread(threading.Thread):
    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap = cap
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self._latest = f
            else:
                time.sleep(0.002)

    def read_latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def stop(self):
        self._stop.set()


# ---------- Helpers ----------
def find_arduino_port():
    for p in serial.tools.list_ports.comports():
        d, n = (p.description or "").lower(), (p.device or "").lower()
        if any(k in d for k in ("arduino", "ch340", "ch341", "wchusbserial", "usb-serial", "usb serial")) \
           or any(k in n for k in ("ttyacm", "ttyusb", "ttych341")):
            return p.device
    for pat in ("/dev/ttyCH341USB*", "/dev/ttyCH341*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def open_camera():
    backend = (cv2.CAP_V4L2 if platform.system() == "Linux"
               else cv2.CAP_DSHOW if platform.system() == "Windows" else 0)
    cap = cv2.VideoCapture(CAMERA_INDEX, backend)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def send_and_wait(ser, cmd, timeout_sec):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    t0 = time.time()
    while (time.time() - t0) < timeout_sec:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "OK":
            return True
        if line == "ERR":
            return False
    return False


# ---------- Main ----------
def main():
    if not os.path.exists(MODEL_PATH):
        print(f"Model not found: {MODEL_PATH}")
        return

    model = YOLO(MODEL_PATH)
    use_cuda = torch.cuda.is_available()
    if TARGET_CLASS not in model.names.values():
        print(f"Class '{TARGET_CLASS}' not in model")
        return
    target_id = next(k for k, v in model.names.items() if v == TARGET_CLASS)

    cap = open_camera()
    if cap is None:
        print("Camera failed")
        return
    cam = CameraThread(cap)
    cam.start()

    port = find_arduino_port()
    if port is None:
        print("No Arduino")
        cam.stop(); cap.release()
        return
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(2.0)
    ser.reset_input_buffer()

    base, theta = BASE_HOME, CAM_HOME
    ser.write(b"home\n")

    sm_x = sm_y = None
    centered_since = None
    last_base_cmd_t = 0.0
    last_cam_cmd_t = 0.0
    pick_armed = False
    waiting_for_grab_align = False

    try:
        while True:
            frame = cam.read_latest()
            if frame is None:
                time.sleep(0.005)
                continue

            now = time.time()
            h, w = frame.shape[:2]
            cx0, cy0 = w // 2, h // 2

            results = model.predict(
                frame, imgsz=INFER_IMGSZ, conf=CONF_THRESH, classes=[target_id],
                device=0 if use_cuda else "cpu", half=use_cuda, verbose=False,
            )[0]

            best = None
            for box in results.boxes:
                if int(box.cls[0]) != target_id:
                    continue
                conf = float(box.conf[0])
                if best is None or conf > best["conf"]:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    best = {"conf": conf, "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                            "cx": (x1 + x2) // 2, "cy": (y1 + y2) // 2}

            cv2.line(frame, (cx0, 0), (cx0, h), (255, 0, 0), 1)
            cv2.line(frame, (0, cy0), (w, cy0), (255, 0, 0), 1)

            centered = False
            if best is not None:
                if sm_x is None:
                    sm_x, sm_y = float(best["cx"]), float(best["cy"])
                else:
                    sm_x = (1 - EMA_ALPHA) * sm_x + EMA_ALPHA * best["cx"]
                    sm_y = (1 - EMA_ALPHA) * sm_y + EMA_ALPHA * best["cy"]
                cv2.rectangle(frame, (best["x1"], best["y1"]), (best["x2"], best["y2"]),
                              (0, 255, 0), 2)
                cv2.circle(frame, (best["cx"], best["cy"]), 5, (0, 255, 255), -1)
                cv2.putText(frame, f"obj:({best['cx']},{best['cy']})",
                            (best["cx"] + 8, best["cy"] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cv2.circle(frame, (int(sm_x), int(sm_y)), 4, (255, 255, 0), -1)

                err_b = int(sm_x) - cx0
                err_c = cy0 - int(sm_y)
                centered = abs(err_b) <= DEADZONE_PX and abs(err_c) <= DEADZONE_PX

                if (now - last_base_cmd_t) >= BASE_CMD_GAP_SEC:
                    if abs(err_b) > DEADZONE_PX:
                        step_b = clamp(-err_b * KP, -MAX_STEP_DEG, MAX_STEP_DEG)
                        if 0.0 < abs(step_b) < 1.0:
                            step_b = -1.0 if step_b < 0 else 1.0
                        nb = clamp(base + int(step_b), BASE_MIN, BASE_MAX)
                        if nb != base:
                            base = nb
                            ser.write(f"b {base}\n".encode())

                    last_base_cmd_t = now

                if (now - last_cam_cmd_t) >= CMD_GAP_SEC:
                    if abs(err_c) > DEADZONE_PX:
                        step_c = clamp(err_c * KP, -MAX_STEP_DEG, MAX_STEP_DEG)
                        if 0.0 < abs(step_c) < 1.0:
                            step_c = -1.0 if step_c < 0 else 1.0
                        nt = clamp(theta + int(step_c), CAM_MIN, CAM_MAX)
                        if nt != theta:
                            theta = nt
                            ser.write(f"c {theta}\n".encode())
                    last_cam_cmd_t = now
            else:
                sm_x = sm_y = None

            centered_since = (centered_since or now) if centered else None
            aligned = centered and centered_since and (now - centered_since) >= ALIGN_HOLD_SEC

            status = "TRACK"
            if waiting_for_grab_align:
                status = "PREPICK_ALIGN" if not aligned else "PREPICK_ALIGNED"
            elif pick_armed:
                status = "PICK_ARMED"
            elif aligned:
                status = "ALIGNED"

            cv2.putText(frame, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if aligned else (0, 200, 200), 2)
            cv2.putText(frame, "p: pick  q: quit", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 2)
            cv2.imshow("Auto Pick", frame)

            if waiting_for_grab_align and aligned:
                if send_and_wait(ser, "grab", GRAB_TIMEOUT_SEC):
                    waiting_for_grab_align = False
                    pick_armed = False
                else:
                    waiting_for_grab_align = False
                    pick_armed = False
                centered_since = None
                t_now = time.time()
                last_base_cmd_t = t_now
                last_cam_cmd_t = t_now

            key = cv2.waitKey(1) & 0xFF
            if key == ord('p'):
                pick_armed = True
                if aligned:
                    if send_and_wait(ser, "prepick", PREPICK_TIMEOUT_SEC):
                        waiting_for_grab_align = True
                    else:
                        pick_armed = False
                        waiting_for_grab_align = False
                    centered_since = None
                    t_now = time.time()
                    last_base_cmd_t = t_now
                    last_cam_cmd_t = t_now
            if key == ord('q'):
                break
    finally:
        cam.stop()
        cap.release()
        cv2.destroyAllWindows()
        try:
            ser.write(b"home\n")
            time.sleep(0.5)
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
