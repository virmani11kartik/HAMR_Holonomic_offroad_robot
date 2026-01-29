#include <Wire.h>
#include <Adafruit_VL53L0X.h>


#define SDA_PIN 36
#define SCL_PIN 35
#define XSHUT_1 37
#define XSHUT_2 38
#define XSHUT_3 39
#define XSHUT_4 40
#define ADDR_1 0x30
#define ADDR_2 0x31
#define ADDR_3 0x32
#define ADDR_4 0x33

Adafruit_VL53L0X tof1;
Adafruit_VL53L0X tof2;
Adafruit_VL53L0X tof3;
Adafruit_VL53L0X tof4;

void readSensor(Adafruit_VL53L0X &lox, const char *label) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  Serial.print(label);
  Serial.print(": ");
  if (measure.RangeStatus != 4) {
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }
}

bool initSensor(Adafruit_VL53L0X &lox, int xshutPin, int newAddr) {
  digitalWrite(xshutPin, HIGH);  
  delay(50);
  if (!lox.begin(0x29, false, &Wire)) {
    return false;
  }
  lox.setAddress(newAddr);      
  delay(10);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  delay(10);
  if (!initSensor(tof1, XSHUT_1, ADDR_1)) Serial.println("Sensor 1 failed");
  if (!initSensor(tof2, XSHUT_2, ADDR_2)) Serial.println("Sensor 2 failed");
  if (!initSensor(tof3, XSHUT_3, ADDR_3)) Serial.println("Sensor 3 failed");
  if (!initSensor(tof4, XSHUT_4, ADDR_4)) Serial.println("Sensor 4 failed");

  Serial.println("All sensors initialized.");
}

void loop() {
  readSensor(tof1, "TOF1");
  readSensor(tof2, "TOF2");
  readSensor(tof3, "TOF3");
  readSensor(tof4, "TOF4");
  Serial.println("----");
  delay(300);
}
