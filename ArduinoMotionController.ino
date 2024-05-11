#define LEFT_BUTTON 12
#define RESET_BUTTON 11
#define RIGHT_BUTTON 10

#define LOOP_TIME 20

// Struct holding information about the current state of the controller
struct ControllerInfo {
  bool leftButton;
  bool rightButton;
};

// Setup all pin modes
void setPinModes() {
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
}

void setControllerInfo(ControllerInfo * cInfo) {
  cInfo->leftButton = digitalRead(LEFT_BUTTON) == LOW;
  cInfo->rightButton = digitalRead(RIGHT_BUTTON) == LOW;
}

void outputControllerInfo(ControllerInfo * cInfo) {
  Serial.print("L: ");
  Serial.print(cInfo->leftButton);
  Serial.print(", R: ");
  Serial.print(cInfo->rightButton);
  Serial.println();
}

// Init controller info
ControllerInfo controllerInfo = {false, false};

void setup() {
  setPinModes();

  Serial.begin(9600);
}

void loop() {
  setControllerInfo(&controllerInfo);

  outputControllerInfo(&controllerInfo);

  delay(LOOP_TIME);
}
