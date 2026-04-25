#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define MUX_ADDR 0x70

Adafruit_VL53L0X tof_grip = Adafruit_VL53L0X(); // Channel 5
Adafruit_VL53L0X tof_cam = Adafruit_VL53L0X();  // Channel 6

void selectBus(uint8_t bus)
{
  if (bus > 7)
    return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  selectBus(2);
  if (!tof_grip.begin())
  {
    Serial.println(F("Failed to boot tof_grip (Channel 5)"));
  } else {
    tof_grip.setMeasurementTimingBudgetMicroSeconds(20000);
  }

  selectBus(3);
  if (!tof_cam.begin())
  {
    Serial.println(F("Failed to boot tof_cam (Channel 6)"));
  } else {
    tof_cam.setMeasurementTimingBudgetMicroSeconds(20000);
  }

  Serial.println(F("Sensors Initialized."));
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measure_grip;
  selectBus(2);
  tof_grip.rangingTest(&measure_grip, false);
  Serial.print("GRIP:");
  if (measure_grip.RangeStatus != 4) {
    Serial.print(measure_grip.RangeMilliMeter);
  } else {
    Serial.print("ERROR");
  }

  Serial.print(" | ");

  VL53L0X_RangingMeasurementData_t measure_cam;
  selectBus(3);
  tof_cam.rangingTest(&measure_cam, false);
  Serial.print("DIST:");
  if (measure_cam.RangeStatus != 4) {
    Serial.println(measure_cam.RangeMilliMeter);
  } else {
    Serial.println("ERROR");
  }
}
