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

// Other constants
#define LOOP_TIME 40
#define SMOOTHING_ITER 10

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

// Current offsets of the gyro
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

// FUNCTIONS

// Sets up everything needed for MPU
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

// Reads the neccesary values and stores them into the cInfo struct
void setControllerInfo(ControllerInfo* cInfo) {
  cInfo->leftButton = digitalRead(LEFT_BUTTON) == LOW;
  cInfo->rightButton = digitalRead(RIGHT_BUTTON) == LOW;

  getRotation(&(cInfo->rotX), &(cInfo->rotY), &(cInfo->rotZ));
}

// Outputs the content of cInfo struct into the Serial
// Format: "L: 1/0, R: 1/0, X: val, Y: val, Z: val"
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

// Sets the current values of the gyro as the offsets
void setOffset() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.dmpGetEuler(euler, &quat);

    offsetX = euler[0] * 180/M_PI;
    offsetY = euler[1] * 180/M_PI;
    offsetZ = euler[2] * 180/M_PI;
  }
}


float totalX = 0;
float totalY = 0;
float totalZ = 0;
int iters = 0;
// Stores the rotation of the gyro into the x, y, z variables. Also applies simple smoothing (using the variables above)
void getRotation(float* x, float* y, float* z) {
  // Reset values
  totalX = 0;
  totalY = 0;
  totalZ = 0;
  iters = 0;

  // Do set ammount of iterations
  for (int i = 0; i < SMOOTHING_ITER; i++) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&quat, fifoBuffer);
      mpu.dmpGetEuler(euler, &quat);

      totalX += (euler[0] * 180/M_PI) - offsetX;
      totalY += (euler[1] * 180/M_PI) - offsetY;
      totalZ += (euler[2] * 180/M_PI) - offsetZ;

      iters++;
    }

    delay(1);
  }

  // Store the average values
  if (iters != 0) {
    *x = totalX / iters;
    *y = totalY / iters;
    *z = totalZ / iters;

    // Set LED on if everything is OK
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    // Set LED off if there is a problem
    digitalWrite(LED_PIN, LOW);
  }
}


// MAIN CODE

void setup() {
  // Turn the LED off = controller is not ready
  digitalWrite(LED_PIN, LOW);

  setPinModes();

  Serial.begin(9600);

  setupMPU();

  // Turn the LED on = controller is ready
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Check for the press of the Reset button
  if (digitalRead(RESET_BUTTON) == LOW) setOffset();

  setControllerInfo(&controllerInfo);

  outputControllerInfo(&controllerInfo);

  delay(LOOP_TIME);
}
