#include <AccelStepper.h>
#define PI 3.14159265358979323846
const int numPoints = 100;
float targetPositions[numPoints][3];
AccelStepper motorX(1, 2, 3);
AccelStepper motorY(1, 4, 5);
AccelStepper motorZ(1, 6, 7);
long MAX_SPEED_X = 1000;
long MAX_SPEED_Y = 1000;
long MAX_SPEED_Z = 1000;
long MAX_Acc = 1000;
const float e = 150;
const float f = 920;
const float re = 540;
const float rf = 200;
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;
const float sin120 = sqrt3 / 2.0;
const float cos120 = -0.5;
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1 / sqrt3;
float theta1, theta2, theta3;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = {0};
float floatFromPC1 = 0.0;
float floatFromPC2 = 0.0;
float floatFromPC3 = 0.0;
boolean newData = false;

void setup() {
  motorX.setMaxSpeed(MAX_SPEED_X);
  motorX.setAcceleration(MAX_Acc);
  motorY.setMaxSpeed(MAX_SPEED_Y);
  motorY.setAcceleration(MAX_Acc);
  motorZ.setMaxSpeed(MAX_SPEED_Z);
  motorZ.setAcceleration(MAX_Acc);
  Serial.begin(38400);
}

void loop() {
  recvWithStartEndMarkers();
  if (newData) {
    parseData();
    float positions[] = {floatFromPC1, floatFromPC2, floatFromPC3};
    showParsedData(positions);
    newData = false;
  }
}

void runAllSimultaneously(float positions[]) {
  motorX.moveTo(positions[0]);
  motorY.moveTo(positions[1]);
  motorZ.moveTo(positions[2]);
  while (motorX.distanceToGo() != 0 || motorY.distanceToGo() != 0 || motorZ.distanceToGo() != 0) {
    motorX.run();
    motorY.run();
    motorZ.run();
  }
}

void showParsedData(float positions[]) {
  int inverseStatus = delta_calcInverse(positions[0], positions[1], positions[2], theta1, theta2, theta3);
  float tempArray[] = {theta1 * 4.4445 * 3, theta2 * 4.4445 * 3, theta3 * 4.4445 * 3};
  runAllSimultaneously(tempArray);
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  strcpy(tempChars, receivedChars);
  char * strtokIndx;

  strtokIndx = strtok(tempChars, ",");
  floatFromPC1 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  floatFromPC2 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  floatFromPC3 = atof(strtokIndx);
}

int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  float y1 = -0.5 * 0.57735 * f;
  y0 -= 0.5 * 0.57735 * e;
  float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;
  float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
  if (d < 0) return -1;
  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1);
  float zj = a + b * yj;
  theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
  return 0;
}

int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  theta1 = theta2 = theta3 = 0;
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, theta2);
  if (status == 0) status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, theta3);
  return status;
}
