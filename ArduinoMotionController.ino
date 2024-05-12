#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// PINS
#define LEFT_BUTTON 12
#define RESET_BUTTON 11
#define RIGHT_BUTTON 10
#define LED_PIN 13

#define LOOP_TIME 20

// The usage of MPU6050 is mostly taken from MPU6050 library and its examples by ElectronicCats:
// https://github.com/ElectronicCats/mpu6050/tree/master
// Declare MPU and its control variables
MPU6050 mpu;
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
// Quaternion and it's Euler representation
Quaternion quat;
float euler[3];

// STRUCTS

// Struct holding information about the current state of the controller
struct ControllerInfo {
  bool leftButton;
  bool rightButton;
  float rotX;
  float rotY;
  float rotZ;
};

// Init controller info
ControllerInfo controllerInfo = { false, false, 0, 0, 0 };
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

// FUNCTIONS

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000);

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = 42;
  while (devStatus != 0) {
    // load and configure the DMP
    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();
  }

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    Serial.println("DMP Initialization failed");
  }
}

// Setup all pin modes
void setPinModes() {
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
}

void setControllerInfo(ControllerInfo* cInfo) {
  cInfo->leftButton = digitalRead(LEFT_BUTTON) == LOW;
  cInfo->rightButton = digitalRead(RIGHT_BUTTON) == LOW;

  getRotation(&(cInfo->rotX), &(cInfo->rotY), &(cInfo->rotZ));
}

void outputControllerInfo(ControllerInfo* cInfo) {
  Serial.print("L: ");
  Serial.print(cInfo->leftButton);
  Serial.print(", R: ");
  Serial.print(cInfo->rightButton);
  Serial.print(", X: ");
  Serial.print(cInfo->rotX);
  Serial.print(", Y: ");
  Serial.print(cInfo->rotY);
  Serial.print(", Z: ");
  Serial.print(cInfo->rotZ);
  Serial.println();
}

void setOffset() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.dmpGetEuler(euler, &quat);

    offsetX = euler[0] * 180/M_PI;
    offsetY = euler[1] * 180/M_PI;
    offsetZ = euler[2] * 180/M_PI;
  }
}

void getRotation(float* x, float* y, float* z) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.dmpGetEuler(euler, &quat);

    *x = (euler[0] * 180/M_PI) - offsetX;
    *y = (euler[1] * 180/M_PI) - offsetY;
    *z = (euler[2] * 180/M_PI) - offsetZ;
  }
}


// MAIN CODE

void setup() {
  digitalWrite(LED_PIN, LOW);

  setPinModes();

  Serial.begin(9600);

  setupMPU();

  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  if (digitalRead(RESET_BUTTON) == LOW) setOffset();

  setControllerInfo(&controllerInfo);

  outputControllerInfo(&controllerInfo);

  delay(LOOP_TIME);
}
