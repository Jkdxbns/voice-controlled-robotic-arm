#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

#define MUX_ADDR 0x70

#define SERVO_CAM  10
#define SERVO_BASE 7
#define SERVO_SHO  6
#define SERVO_ELB  5
#define SERVO_GRIP 3

#define CAM_MIN  35
#define CAM_MAX  120
#define CAM_HOME 40

#define BASE_MIN  0
#define BASE_MAX  90
#define BASE_HOME 45

#define SHO_HOME  90
#define ELB_HOME  10
#define GRIP_HOME 0

#define STEP_DEG 1   // degrees per L/R/U/D step

// Motion smoothing (smaller step + delay = less jerk)
#define SERVO_SMOOTH_STEP_DEG 1
#define SERVO_SMOOTH_DELAY_MS 18

#define TOF_GRIP_CH 2
#define TOF_CAM_CH  3

// Arm geometry in mm (old link lengths)
const float LINK_SHOULDER_TO_ELBOW_MM = 75.0f;
const float LINK_ELBOW_TO_EE_MM = 155.0f;

// Unified shoulder-centered frame used everywhere in this firmware:
//   x: downward (parallel to gravity)
//   y: forward (camera-looking direction)
// New camera conversion constants (mm):
//   x = (L+28)cos(theta) - 74
//   y = (L+28)sin(theta) + 40
const float CAM_L_BIAS_MM = 28.0f;
const float CAM_X_BIAS_MM = -74.0f;
const float CAM_Y_BIAS_MM = 40.0f;

Adafruit_VL53L0X tof_grip = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_cam  = Adafruit_VL53L0X();

Servo servo_cam;
Servo servo_base;
Servo servo_sho;
Servo servo_elb;
Servo servo_grip;

int cam_angle  = CAM_HOME;
int base_angle = BASE_HOME;
int sho_angle  = SHO_HOME;
int elb_angle  = ELB_HOME;
int grip_angle = GRIP_HOME;

static bool inRangeF(float v, float lo, float hi)
{
  return (v >= lo && v <= hi);
}

void selectBus(uint8_t bus)
{
  if (bus > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

// Returns gripper-side TOF reading in mm, or -1 on error.
int readGripDistance()
{
  selectBus(TOF_GRIP_CH);
  VL53L0X_RangingMeasurementData_t m;
  tof_grip.rangingTest(&m, false);
  if (m.RangeStatus == 4) return -1;
  return m.RangeMilliMeter;
}

// Returns cam-side TOF reading in mm, or -1 on error.
int readCamDistance()
{
  selectBus(TOF_CAM_CH);
  VL53L0X_RangingMeasurementData_t m;
  tof_cam.rangingTest(&m, false);
  if (m.RangeStatus == 4) return -1;
  return m.RangeMilliMeter;
}

void setBaseAngle(int a)
{
  if (a < BASE_MIN) a = BASE_MIN;
  if (a > BASE_MAX) a = BASE_MAX;

  int dir = (a >= base_angle) ? 1 : -1;
  int step = SERVO_SMOOTH_STEP_DEG * dir;
  while (base_angle != a) {
    int next = base_angle + step;
    if ((dir > 0 && next > a) || (dir < 0 && next < a)) next = a;
    base_angle = next;
    servo_base.write(base_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

void setCamAngle(int a)
{
  if (a < CAM_MIN) a = CAM_MIN;
  if (a > CAM_MAX) a = CAM_MAX;

  int dir = (a >= cam_angle) ? 1 : -1;
  int step = SERVO_SMOOTH_STEP_DEG * dir;
  while (cam_angle != a) {
    int next = cam_angle + step;
    if ((dir > 0 && next > a) || (dir < 0 && next < a)) next = a;
    cam_angle = next;
    servo_cam.write(cam_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

static void smoothMoveArmSE(int targetSho, int targetElb)
{
  if (targetSho < 0) targetSho = 0; else if (targetSho > 180) targetSho = 180;
  if (targetElb < 0) targetElb = 0; else if (targetElb > 180) targetElb = 180;

  while (sho_angle != targetSho || elb_angle != targetElb) {
    if (sho_angle < targetSho) sho_angle += SERVO_SMOOTH_STEP_DEG;
    else if (sho_angle > targetSho) sho_angle -= SERVO_SMOOTH_STEP_DEG;

    if (elb_angle < targetElb) elb_angle += SERVO_SMOOTH_STEP_DEG;
    else if (elb_angle > targetElb) elb_angle -= SERVO_SMOOTH_STEP_DEG;

    if (sho_angle < 0) sho_angle = 0; else if (sho_angle > 180) sho_angle = 180;
    if (elb_angle < 0) elb_angle = 0; else if (elb_angle > 180) elb_angle = 180;

    servo_sho.write(sho_angle);
    servo_elb.write(elb_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

static void setGripperAngle(int a)
{
  if (a < 0) a = 0;
  if (a > 180) a = 180;

  int dir = (a >= grip_angle) ? 1 : -1;
  int step = SERVO_SMOOTH_STEP_DEG * dir;
  while (grip_angle != a) {
    int next = grip_angle + step;
    if ((dir > 0 && next > a) || (dir < 0 && next < a)) next = a;
    grip_angle = next;
    servo_grip.write(grip_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

static void printStateLine()
{
  Serial.print(F("STATE:"));
  Serial.print(base_angle);
  Serial.print(F(","));
  Serial.print(cam_angle);
  Serial.print(F(","));
  Serial.print(sho_angle);
  Serial.print(F(","));
  Serial.print(elb_angle);
  Serial.print(F(","));
  Serial.println(grip_angle);
}

// Solve one IK branch using the proven mapping style from code_works_new.
static bool solveIKBranch(float x_mm, float y_mm, bool elbowUp,
                          float &q1Deg, float &q2Deg,
                          float &shoulderCmdDeg, float &elbowCmdDeg)
{
  const float L1 = LINK_SHOULDER_TO_ELBOW_MM;
  const float L2 = LINK_ELBOW_TO_EE_MM;

  float c2 = (x_mm * x_mm + y_mm * y_mm - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (c2 < -1.0f || c2 > 1.0f) return false;

  float s2 = sqrtf(1.0f - c2 * c2);
  if (!elbowUp) s2 = -s2;

  q2Deg = atan2f(s2, c2) * 180.0f / PI;
  q1Deg = (atan2f(y_mm, x_mm) - atan2f(L2 * s2, L1 + L2 * c2)) * 180.0f / PI;

  shoulderCmdDeg = SHO_HOME + q1Deg;
  elbowCmdDeg = ELB_HOME + q2Deg;
  return true;
}

static bool moveToXYMM(float x_mm, float y_mm)
{
  float q1A, q2A, shaA, elaA;
  float q1B, q2B, shaB, elaB;

  bool realA = solveIKBranch(x_mm, y_mm, true, q1A, q2A, shaA, elaA);
  bool realB = solveIKBranch(x_mm, y_mm, false, q1B, q2B, shaB, elaB);

  bool validA = realA && inRangeF(shaA, 0.0f, 180.0f) && inRangeF(elaA, 0.0f, 180.0f);
  bool validB = realB && inRangeF(shaB, 0.0f, 180.0f) && inRangeF(elaB, 0.0f, 180.0f);

  Serial.print(F("IK target x_mm="));
  Serial.print(x_mm, 2);
  Serial.print(F(" y_mm="));
  Serial.println(y_mm, 2);

  if (realA) {
    Serial.print(F("A q1="));
    Serial.print(q1A, 2);
    Serial.print(F(" q2="));
    Serial.println(q2A, 2);
  }

  if (realB) {
    Serial.print(F("B q1="));
    Serial.print(q1B, 2);
    Serial.print(F(" q2="));
    Serial.println(q2B, 2);
  }

  if (validA) {
    Serial.print(F("A: shoulder="));
    Serial.print(shaA, 2);
    Serial.print(F(" elbow="));
    Serial.println(elaA, 2);
  } else {
    Serial.println(F("A: invalid"));
  }

  if (validB) {
    Serial.print(F("B: shoulder="));
    Serial.print(shaB, 2);
    Serial.print(F(" elbow="));
    Serial.println(elaB, 2);
  } else {
    Serial.println(F("B: invalid"));
  }

  bool useA = false;
  bool useB = false;

  float curSho = servo_sho.read();
  float curElb = servo_elb.read();

  if (validA && validB) {
    float distA = fabsf(shaA - curSho) + fabsf(elaA - curElb);
    float distB = fabsf(shaB - curSho) + fabsf(elaB - curElb);
    if (distA <= distB) useA = true; else useB = true;
  } else if (validA) {
    useA = true;
  } else if (validB) {
    useB = true;
  }

  if (useA) {
    smoothMoveArmSE((int)roundf(shaA), (int)roundf(elaA));
    Serial.println(F("IK moved using branch A"));
    return true;
  }
  if (useB) {
    smoothMoveArmSE((int)roundf(shaB), (int)roundf(elaB));
    Serial.println(F("IK moved using branch B"));
    return true;
  }

  Serial.println(F("IK unreachable"));
  return false;
}

// Uses new calibration equations with raw camera servo angle theta (deg).
static bool moveUsingTofCam(int baseAngleRaw)
{
  int L = readCamDistance();
  if (L < 0) {
    Serial.println(F("CAMIK:TOF_ERROR"));
    return false;
  }

  float thetaRad = cam_angle * PI / 180.0f;
  float Lb = ((float)L) + CAM_L_BIAS_MM;
  float x_tgt = Lb * cosf(thetaRad) + CAM_X_BIAS_MM;
  float y_tgt = Lb * sinf(thetaRad) + CAM_Y_BIAS_MM;

  setBaseAngle(baseAngleRaw);

  Serial.print(F("CAMIK:L="));
  Serial.print(L);
  Serial.print(F(",theta="));
  Serial.print(cam_angle);
  Serial.print(F(",xy="));
  Serial.print(x_tgt, 2);
  Serial.print(F(","));
  Serial.print(y_tgt, 2);
  Serial.print(F(",z(base)="));
  Serial.println(base_angle);

  return moveToXYMM(x_tgt, y_tgt);
}

// Variant of camik that applies a y-axis offset in mm in the unified frame.
// Unified frame: x=down, y=forward.
static bool moveUsingTofCamOffset(int baseAngleRaw, float yOffsetEqMm)
{
  int L = readCamDistance();
  if (L < 0) {
    Serial.println(F("CAMIKOFF:TOF_ERROR"));
    return false;
  }

  float thetaRad = cam_angle * PI / 180.0f;
  float Lb = ((float)L) + CAM_L_BIAS_MM;
  float x_tgt = Lb * cosf(thetaRad) + CAM_X_BIAS_MM;
  float y_tgt = Lb * sinf(thetaRad) + CAM_Y_BIAS_MM + yOffsetEqMm;

  setBaseAngle(baseAngleRaw);

  Serial.print(F("CAMIKOFF:L="));
  Serial.print(L);
  Serial.print(F(",theta="));
  Serial.print(cam_angle);
  Serial.print(F(",yOff="));
  Serial.print(yOffsetEqMm, 2);
  Serial.print(F(",xy="));
  Serial.print(x_tgt, 2);
  Serial.print(F(","));
  Serial.print(y_tgt, 2);
  Serial.print(F(",z(base)="));
  Serial.println(base_angle);

  return moveToXYMM(x_tgt, y_tgt);
}

// Variant of camik that applies unified-frame x/y offsets in mm.
// Unified frame: x=down, y=forward.
static bool moveUsingTofCamOffsetXY(int baseAngleRaw, float xOffsetEqMm, float yOffsetEqMm)
{
  int L = readCamDistance();
  if (L < 0) {
    Serial.println(F("CAMIKXYOFF:TOF_ERROR"));
    return false;
  }

  float thetaRad = cam_angle * PI / 180.0f;
  float Lb = ((float)L) + CAM_L_BIAS_MM;
  float x_tgt = Lb * cosf(thetaRad) + CAM_X_BIAS_MM + xOffsetEqMm;
  float y_tgt = Lb * sinf(thetaRad) + CAM_Y_BIAS_MM + yOffsetEqMm;

  setBaseAngle(baseAngleRaw);

  Serial.print(F("CAMIKXYOFF:L="));
  Serial.print(L);
  Serial.print(F(",theta="));
  Serial.print(cam_angle);
  Serial.print(F(",xOff="));
  Serial.print(xOffsetEqMm, 2);
  Serial.print(F(",yOff="));
  Serial.print(yOffsetEqMm, 2);
  Serial.print(F(",xy="));
  Serial.print(x_tgt, 2);
  Serial.print(F(","));
  Serial.print(y_tgt, 2);
  Serial.print(F(",z(base)="));
  Serial.println(base_angle);

  return moveToXYMM(x_tgt, y_tgt);
}

// Fixed-step fallback (kept for manual/keystroke control).
void parseDirection(char c)
{
  switch (c) {
    case 'L': case 'l': setBaseAngle(base_angle - STEP_DEG); break;
    case 'R': case 'r': setBaseAngle(base_angle + STEP_DEG); break;
    case 'U': case 'u': setCamAngle(cam_angle + STEP_DEG); break;
    case 'D': case 'd': setCamAngle(cam_angle - STEP_DEG); break;
    default: break;
  }
}

// Line dispatch: "b <angle>", "c <angle>", "s <angle>", "e <angle>", "g <angle>",
// "home", "d" (distances), or single-char L/R/U/D.
void handleLine(char* s)
{
  if (s[0] == 0) return;
  if ((s[0] == 'b' || s[0] == 'B') && s[1] == ' ') {
    setBaseAngle(atoi(s + 2));
  } else if ((s[0] == 'c' || s[0] == 'C') && s[1] == ' ') {
    setCamAngle(atoi(s + 2));
  } else if ((s[0] == 's' || s[0] == 'S') && s[1] == ' ') {
    int a = atoi(s + 2);
    smoothMoveArmSE(a, elb_angle);
  } else if ((s[0] == 'e' || s[0] == 'E') && s[1] == ' ') {
    int a = atoi(s + 2);
    smoothMoveArmSE(sho_angle, a);
  } else if ((s[0] == 'g' || s[0] == 'G') && s[1] == ' ') {
    int a = atoi(s + 2);
    setGripperAngle(a);
  } else if ((s[0] == 'x' || s[0] == 'X') && (s[1] == 'y' || s[1] == 'Y') && s[2] == ' ') {
    // Parse robustly (avoids sscanf float issues on some Arduino builds).
    char *p = s + 3;
    while (*p == ' ') p++;
    char *sep = strchr(p, ' ');
    if (sep != NULL) {
      *sep = 0;
      char *p2 = sep + 1;
      while (*p2 == ' ') p2++;
      if (*p2 != 0) {
        float inX = atof(p);
        float inY = atof(p2);
        moveToXYMM(inX, inY);
      } else {
        Serial.println(F("Usage: xy <x_mm> <y_mm>"));
      }
    } else {
      Serial.println(F("Usage: xy <x_mm> <y_mm>"));
    }
  } else if (!strcasecmp(s, "camik")) {
    moveUsingTofCam(base_angle);
  } else if (!strncasecmp(s, "camikoff ", 9)) {
    // Usage: camikoff <base_angle> <y_offset_eq_mm>
    char *p = s + 9;
    while (*p == ' ') p++;
    char *sep = strchr(p, ' ');
    if (sep != NULL) {
      *sep = 0;
      char *p2 = sep + 1;
      while (*p2 == ' ') p2++;
      if (*p2 != 0) {
        int z = atoi(p);
        float yOff = atof(p2);
        moveUsingTofCamOffset(z, yOff);
      } else {
        Serial.println(F("Usage: camikoff <base> <y_offset_eq_mm>"));
      }
    } else {
      Serial.println(F("Usage: camikoff <base> <y_offset_eq_mm>"));
    }
  } else if (!strncasecmp(s, "camikxyoff ", 10)) {
    // Usage: camikxyoff <base_angle> <x_offset_eq_mm> <y_offset_eq_mm>
    char *p = s + 10;
    while (*p == ' ') p++;
    char *sep1 = strchr(p, ' ');
    if (sep1 != NULL) {
      *sep1 = 0;
      char *p2 = sep1 + 1;
      while (*p2 == ' ') p2++;
      char *sep2 = strchr(p2, ' ');
      if (sep2 != NULL) {
        *sep2 = 0;
        char *p3 = sep2 + 1;
        while (*p3 == ' ') p3++;
        if (*p3 != 0) {
          int z = atoi(p);
          float xOff = atof(p2);
          float yOff = atof(p3);
          moveUsingTofCamOffsetXY(z, xOff, yOff);
        } else {
          Serial.println(F("Usage: camikxyoff <base> <x_off_eq_mm> <y_off_eq_mm>"));
        }
      } else {
        Serial.println(F("Usage: camikxyoff <base> <x_off_eq_mm> <y_off_eq_mm>"));
      }
    } else {
      Serial.println(F("Usage: camikxyoff <base> <x_off_eq_mm> <y_off_eq_mm>"));
    }
  } else if ((s[0] == 'L' || s[0] == 'l') && s[1] == ' ') {
    // Debug helper: use provided distance with current theta(cam servo).
    int dist = atoi(s + 2);
    if (dist > 0) {
      int saved = readCamDistance();
      (void)saved; // keep sensor read path unchanged elsewhere
      float thetaRad = cam_angle * PI / 180.0f;
      float Lb = ((float)dist) + CAM_L_BIAS_MM;
      float x_tgt = Lb * cosf(thetaRad) + CAM_X_BIAS_MM;
      float y_tgt = Lb * sinf(thetaRad) + CAM_Y_BIAS_MM;
      Serial.print(F("LDBG:L=")); Serial.print(dist);
      Serial.print(F(" xy=")); Serial.print(x_tgt, 2); Serial.print(F(",")); Serial.println(y_tgt, 2);
      moveToXYMM(x_tgt, y_tgt);
    } else {
      Serial.println(F("Usage: L <dist_mm>"));
    }
  } else if ((s[0] == 'z' || s[0] == 'Z') && s[1] == ' ') {
    int z = atoi(s + 2);
    moveUsingTofCam(z);
  } else if (!strcasecmp(s, "home")) {
    setBaseAngle(BASE_HOME);
    setCamAngle(CAM_HOME);
    smoothMoveArmSE(SHO_HOME, ELB_HOME);
    setGripperAngle(GRIP_HOME);
    printStateLine();
  } else if (!strcasecmp(s, "state")) {
    printStateLine();
  } else if ((s[0] == 'd' || s[0] == 'D') && s[1] == 0) {
    int g = readGripDistance();
    int c = readCamDistance();
    Serial.print(F("DISTS:"));
    if (g < 0) Serial.print(F("ERR")); else Serial.print(g);
    Serial.print(F(","));
    if (c < 0) Serial.println(F("ERR")); else Serial.println(c);
  } else if (s[1] == 0) {
    parseDirection(s[0]);
  }
}

char linebuf[32];
uint8_t linepos = 0;

void setup()
{
  Serial.begin(115200);

  // Pre-seed home angles before attach so the very first pulse is at home,
  // not the library default of 1500us / 90 deg.
  servo_cam.write(CAM_HOME);
  servo_base.write(BASE_HOME);
  servo_sho.write(SHO_HOME);
  servo_elb.write(ELB_HOME);
  servo_grip.write(GRIP_HOME);

  base_angle = BASE_HOME;
  cam_angle = CAM_HOME;
  sho_angle = SHO_HOME;
  elb_angle = ELB_HOME;
  grip_angle = GRIP_HOME;

  // Stagger attaches so 5 servos don't brown-out the rail at once.
  servo_cam.attach(SERVO_CAM);    delay(80);
  servo_base.attach(SERVO_BASE);  delay(80);
  servo_sho.attach(SERVO_SHO);    delay(80);
  servo_elb.attach(SERVO_ELB);    delay(80);
  servo_grip.attach(SERVO_GRIP);  delay(80);

  Wire.begin();
  Wire.setClock(400000);

  selectBus(TOF_GRIP_CH);
  if (!tof_grip.begin()) {
    Serial.println(F("Failed to boot tof_grip (ch5)"));
  } else {
    tof_grip.setMeasurementTimingBudgetMicroSeconds(20000);
  }

  selectBus(TOF_CAM_CH);
  if (!tof_cam.begin()) {
    Serial.println(F("Failed to boot tof_cam (ch6)"));
  } else {
    tof_cam.setMeasurementTimingBudgetMicroSeconds(20000);
  }

  Serial.println(F("READY"));
}

void loop()
{
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      linebuf[linepos] = 0;
      handleLine(linebuf);
      linepos = 0;
    } else if (linepos < sizeof(linebuf) - 1) {
      linebuf[linepos++] = c;
    } else {
      linepos = 0;   // overflow -> discard
    }
  }
}
