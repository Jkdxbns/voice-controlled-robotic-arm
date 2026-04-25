#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

#define MUX_ADDR 0x70

#define SERVO_CAM 10
#define SERVO_BASE 7
#define SERVO_SHO 6
#define SERVO_ELB 5
#define SERVO_GRIP 3

#define TOF_GRIP_CH 2
#define TOF_CAM_CH 3

#define CAM_MIN 35
#define CAM_MAX 120
#define CAM_HOME 40

#define BASE_MIN 0
#define BASE_MAX 90
#define BASE_HOME 45

#define SHO_HOME 90
#define ELB_HOME 10
#define GRIP_HOME 0

#define GRIP_OPEN_ANGLE 75
#define GRIP_CLOSE_ANGLE 10

#define SERVO_SMOOTH_STEP_DEG 2
#define SERVO_SMOOTH_DELAY_MS 8
#define SHOULDER_RATE_NUM 3
#define SHOULDER_RATE_DEN 5

const float L1_MM = 75.0f;
const float L2_MM = 155.0f;

const float CAM_L_BIAS_MM = 28.0f;
const float CAM_X_BIAS_MM = -74.0f;
const float CAM_Y_BIAS_MM = 40.0f;

const float STANDOFF_Y_MM = 90.0f;
const float GRASP_X_OFFSET_MM = 20.0f;
const float GRASP_Y_OFFSET_MM = -15.0f;
const float HOME_X_MM = 230.0f;
const float HOME_Y_MM = 0.0f;

const int STAGE_SETTLE_MS = 220;
const int GRIP_SETTLE_MS = 180;

const bool ENABLE_GRIP_TOF_INIT = false;

Adafruit_VL53L0X tof_grip;
Adafruit_VL53L0X tof_cam;

Servo servo_cam;
Servo servo_base;
Servo servo_sho;
Servo servo_elb;
Servo servo_grip;

int cam_angle = CAM_HOME;
int base_angle = BASE_HOME;
int sho_angle = SHO_HOME;
int elb_angle = ELB_HOME;
int grip_angle = GRIP_HOME;

char linebuf[48];
uint8_t linepos = 0;

void selectBus(uint8_t bus)
{
  if (bus > 7)
    return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

int readCamDistance()
{
  selectBus(TOF_CAM_CH);
  VL53L0X_RangingMeasurementData_t m;
  tof_cam.rangingTest(&m, false);
  if (m.RangeStatus == 4)
    return -1;
  return m.RangeMilliMeter;
}

static bool inRangeF(float v, float lo, float hi)
{
  return (v >= lo && v <= hi);
}

void setBaseAngle(int a)
{
  if (a < BASE_MIN)
    a = BASE_MIN;
  if (a > BASE_MAX)
    a = BASE_MAX;

  int dir = (a >= base_angle) ? 1 : -1;
  int step = SERVO_SMOOTH_STEP_DEG * dir;
  while (base_angle != a)
  {
    int next = base_angle + step;
    if ((dir > 0 && next > a) || (dir < 0 && next < a))
      next = a;
    base_angle = next;
    servo_base.write(base_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

void setCamAngle(int a)
{
  if (a < CAM_MIN)
    a = CAM_MIN;
  if (a > CAM_MAX)
    a = CAM_MAX;

  int dir = (a >= cam_angle) ? 1 : -1;
  int step = SERVO_SMOOTH_STEP_DEG * dir;
  while (cam_angle != a)
  {
    int next = cam_angle + step;
    if ((dir > 0 && next > a) || (dir < 0 && next < a))
      next = a;
    cam_angle = next;
    servo_cam.write(cam_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

void setGripperAngle(int a)
{
  if (a < 0)
    a = 0;
  if (a > 180)
    a = 180;

  int dir = (a >= grip_angle) ? 1 : -1;
  int step = SERVO_SMOOTH_STEP_DEG * dir;
  while (grip_angle != a)
  {
    int next = grip_angle + step;
    if ((dir > 0 && next > a) || (dir < 0 && next < a))
      next = a;
    grip_angle = next;
    servo_grip.write(grip_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

static bool solveIKBranch(float x_mm, float y_mm, bool elbowUp,
                          float &shoCmdDeg, float &elbCmdDeg)
{
  float c2 = (x_mm * x_mm + y_mm * y_mm - L1_MM * L1_MM - L2_MM * L2_MM) /
             (2.0f * L1_MM * L2_MM);
  if (c2 < -1.0f || c2 > 1.0f)
    return false;

  float s2 = sqrtf(1.0f - c2 * c2);
  if (!elbowUp)
    s2 = -s2;

  float q2Deg = atan2f(s2, c2) * 180.0f / PI;
  float q1Deg = (atan2f(y_mm, x_mm) - atan2f(L2_MM * s2, L1_MM + L2_MM * c2)) * 180.0f / PI;

  shoCmdDeg = SHO_HOME + q1Deg;
  elbCmdDeg = ELB_HOME + q2Deg;
  return true;
}

void smoothMoveArmSE(int targetSho, int targetElb)
{
  if (targetSho < 0)
    targetSho = 0;
  else if (targetSho > 180)
    targetSho = 180;
  if (targetElb < 0)
    targetElb = 0;
  else if (targetElb > 180)
    targetElb = 180;

  int shoulder_phase = 0;

  while (sho_angle != targetSho || elb_angle != targetElb)
  {
    shoulder_phase += SHOULDER_RATE_NUM;
    if (shoulder_phase >= SHOULDER_RATE_DEN)
    {
      shoulder_phase -= SHOULDER_RATE_DEN;
      if (sho_angle < targetSho)
      {
        int next = sho_angle + SERVO_SMOOTH_STEP_DEG;
        if (next > targetSho)
          next = targetSho;
        sho_angle = next;
      }
      else if (sho_angle > targetSho)
      {
        int next = sho_angle - SERVO_SMOOTH_STEP_DEG;
        if (next < targetSho)
          next = targetSho;
        sho_angle = next;
      }
    }

    if (elb_angle < targetElb)
    {
      int next = elb_angle + SERVO_SMOOTH_STEP_DEG;
      if (next > targetElb)
        next = targetElb;
      elb_angle = next;
    }
    else if (elb_angle > targetElb)
    {
      int next = elb_angle - SERVO_SMOOTH_STEP_DEG;
      if (next < targetElb)
        next = targetElb;
      elb_angle = next;
    }

    servo_sho.write(sho_angle);
    servo_elb.write(elb_angle);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

bool moveToXYMM(float x_mm, float y_mm)
{
  float shoA, elbA, shoB, elbB;
  bool realA = solveIKBranch(x_mm, y_mm, true, shoA, elbA);
  bool realB = solveIKBranch(x_mm, y_mm, false, shoB, elbB);

  bool validA = realA && inRangeF(shoA, 0.0f, 180.0f) && inRangeF(elbA, 0.0f, 180.0f);
  bool validB = realB && inRangeF(shoB, 0.0f, 180.0f) && inRangeF(elbB, 0.0f, 180.0f);

  bool useA = false;
  bool useB = false;

  if (validA && validB)
  {
    float distA = fabsf(shoA - sho_angle) + fabsf(elbA - elb_angle);
    float distB = fabsf(shoB - sho_angle) + fabsf(elbB - elb_angle);
    if (distA <= distB)
      useA = true;
    else
      useB = true;
  }
  else if (validA)
  {
    useA = true;
  }
  else if (validB)
  {
    useB = true;
  }

  if (useA)
  {
    smoothMoveArmSE((int)roundf(shoA), (int)roundf(elbA));
    return true;
  }
  if (useB)
  {
    smoothMoveArmSE((int)roundf(shoB), (int)roundf(elbB));
    return true;
  }

  return false;
}

void goSafeHome()
{
  setBaseAngle(BASE_HOME);
  setCamAngle(CAM_HOME);
  smoothMoveArmSE(SHO_HOME, ELB_HOME);
  setGripperAngle(GRIP_HOME);
}

bool computeTargetFromCam(int L_mm, int theta_deg, float &x_out, float &y_out)
{
  if (L_mm <= 0)
    return false;
  float thetaRad = theta_deg * PI / 180.0f;
  float Lb = ((float)L_mm) + CAM_L_BIAS_MM;
  x_out = Lb * cosf(thetaRad) + CAM_X_BIAS_MM;
  y_out = Lb * sinf(thetaRad) + CAM_Y_BIAS_MM;
  return true;
}

bool doPickAndPlace()
{
  int L = readCamDistance();
  if (L < 0)
    return false;

  float x_obj = 0.0f, y_obj = 0.0f;
  if (!computeTargetFromCam(L, cam_angle, x_obj, y_obj))
    return false;

  float y_obj_pre = y_obj - STANDOFF_Y_MM;

  setGripperAngle(GRIP_OPEN_ANGLE);
  delay(GRIP_SETTLE_MS);

  if (!moveToXYMM(x_obj, y_obj_pre))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  if (!moveToXYMM(x_obj + GRASP_X_OFFSET_MM, y_obj + GRASP_Y_OFFSET_MM))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  setGripperAngle(GRIP_CLOSE_ANGLE);
  delay(GRIP_SETTLE_MS);

  if (!moveToXYMM(x_obj, y_obj_pre))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  if (!moveToXYMM(HOME_X_MM, HOME_Y_MM))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  return true;
}

bool doPrePick()
{
  int L = readCamDistance();
  if (L < 0)
    return false;

  float x_obj = 0.0f, y_obj = 0.0f;
  if (!computeTargetFromCam(L, cam_angle, x_obj, y_obj))
    return false;

  float y_obj_pre = y_obj - STANDOFF_Y_MM;

  setGripperAngle(GRIP_OPEN_ANGLE);
  delay(GRIP_SETTLE_MS);

  if (!moveToXYMM(x_obj, y_obj_pre))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  return true;
}

bool doGrabFromCurrentCam()
{
  int L = readCamDistance();
  if (L < 0)
    return false;

  float x_obj = 0.0f, y_obj = 0.0f;
  if (!computeTargetFromCam(L, cam_angle, x_obj, y_obj))
    return false;

  float y_obj_pre = y_obj - STANDOFF_Y_MM;

  if (!moveToXYMM(x_obj + GRASP_X_OFFSET_MM, y_obj + GRASP_Y_OFFSET_MM))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  setGripperAngle(GRIP_CLOSE_ANGLE);
  delay(GRIP_SETTLE_MS);

  if (!moveToXYMM(x_obj, y_obj_pre))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  if (!moveToXYMM(HOME_X_MM, HOME_Y_MM))
  {
    goSafeHome();
    return false;
  }
  delay(STAGE_SETTLE_MS);

  return true;
}

void handleLine(char *s)
{
  if (s[0] == 0)
    return;

  if (!strcasecmp(s, "pick") || !strcasecmp(s, "pp"))
  {
    if (doPickAndPlace())
      Serial.println(F("OK"));
    else
      Serial.println(F("ERR"));
    return;
  }

  if (!strcasecmp(s, "prepick"))
  {
    if (doPrePick())
      Serial.println(F("OK"));
    else
      Serial.println(F("ERR"));
    return;
  }

  if (!strcasecmp(s, "grab"))
  {
    if (doGrabFromCurrentCam())
      Serial.println(F("OK"));
    else
      Serial.println(F("ERR"));
    return;
  }

  if (!strcasecmp(s, "home"))
  {
    goSafeHome();
    return;
  }

  if ((s[0] == 'b' || s[0] == 'B') && s[1] == ' ')
  {
    setBaseAngle(atoi(s + 2));
    return;
  }

  if ((s[0] == 'c' || s[0] == 'C') && s[1] == ' ')
  {
    setCamAngle(atoi(s + 2));
    return;
  }

  if ((s[0] == 'g' || s[0] == 'G') && s[1] == ' ')
  {
    setGripperAngle(atoi(s + 2));
    return;
  }
}

void setup()
{
  Serial.begin(115200);

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

  servo_cam.attach(SERVO_CAM);
  delay(80);
  servo_base.attach(SERVO_BASE);
  delay(80);
  servo_sho.attach(SERVO_SHO);
  delay(80);
  servo_elb.attach(SERVO_ELB);
  delay(80);
  servo_grip.attach(SERVO_GRIP);
  delay(80);

  Wire.begin();
  Wire.setClock(400000);

  if (ENABLE_GRIP_TOF_INIT)
  {
    selectBus(TOF_GRIP_CH);
    if (tof_grip.begin())
      tof_grip.setMeasurementTimingBudgetMicroSeconds(20000);
  }

  selectBus(TOF_CAM_CH);
  if (tof_cam.begin())
    tof_cam.setMeasurementTimingBudgetMicroSeconds(20000);
}

void loop()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      linebuf[linepos] = 0;
      handleLine(linebuf);
      linepos = 0;
    }
    else if (linepos < sizeof(linebuf) - 1)
    {
      linebuf[linepos++] = c;
    }
    else
    {
      linepos = 0;
    }
  }
}
