#include <QTRSensors.h>
#include <EEPROM.h>

const int buttonPin = 2;
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
const int ledPinRight = 12;
const int ledPinLeft = 13;

int m1Speed = 0;
int m2Speed = 0;

float kp = 10;
float ki = 0;
float kd = 0;

float p = 0;
float i = 0;
float d = 0;

float error = 0;
float lastError = 0;

float pThreshold = 30;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 255;

const int calibrateSpeed = 50;
const unsigned long calibrateTime = 250;
const int calibrateMoves = 24;
QTRSensors qtr;

byte rightLedState = HIGH;
byte leftLedState = HIGH;

const int blinkDuration = 500;
unsigned long blinkStartLeft = 0;
unsigned long blinkStartRight = 0;


const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };
const unsigned short int sensorAddress[] = { 0, 32, 64, 96, 128, 160 };
byte buttonState = HIGH;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned int debounceDelay = 50;
byte lastReading = LOW;
byte reading = LOW;


void setup() {

  // pinMode setup
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  pinMode(ledPinRight, OUTPUT);
  pinMode(ledPinLeft, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  digitalWrite(LED_BUILTIN, HIGH);

  digitalWrite(ledPinLeft, leftLedState);
  digitalWrite(ledPinRight, rightLedState);

  getSensorsValues();

  digitalWrite(LED_BUILTIN, LOW);
  setMotorSpeed(0, 0);
}

void loop() {
  debounce();

  blink(rightLedState, blinkStartRight);
  blink(leftLedState, blinkStartLeft);

  digitalWrite(ledPinLeft, leftLedState);
  digitalWrite(ledPinRight, rightLedState);

  float error = map(qtr.readLineBlack(sensorValues), 0, 5000, -pThreshold, pThreshold);

  int motorSpeedDiff = pidControl(error);

  CalculateSpeed(error, motorSpeedDiff);

  setMotorSpeed(m1Speed, m2Speed);
  // debugging();
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}

void debugging() {
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("M1 speed: ");
  Serial.println(m1Speed);

  Serial.print("M2 speed: ");
  Serial.println(m2Speed);
}

void customCalibrate() {
  int ct = 0;
  int state = 0;
  while (ct <= 6) {
    qtr.calibrate();
    qtr.read(sensorValues);
    if (state == 0) {  // RIGHT
      if (sensorValues[0] < 700) {
        setMotorSpeed(70, -70);
      } else {
        state = 1;
      }
    }
    if (state == 1) {  // LEFT
      if (sensorValues[5] < 700) {
        setMotorSpeed(-70, 70);
      } else {
        state = 0;
        ct++;
      }
    }
  }
  // qtr.read(sensorValues);
  putSensorsValues();
}

int pidControl(float error) {
  int errorAbs = abs(error);

  p = error;
  i = i + error;
  d = error - lastError;

  if (errorAbs >= 0 && errorAbs < 5) {
    kd = 2;
  }
  if (errorAbs >= 5 && errorAbs < 15) {
    kd = 4;
  }  
  if (errorAbs >= 15 && errorAbs < 20) {
    kd = 6;
  }
  // kd = 0;
  ki = 0.005;

  int motorSpeedDiff = kp * p + ki * i + kd * d;

  return motorSpeedDiff;
}

void CalculateSpeed(float error, int motorSpeedDiff) {
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeedDiff;
  } else if (error > 0) {
    m2Speed -= motorSpeedDiff;
  }

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
}

void blink(byte &ledState, unsigned long &blinkStart) {
  if (millis() - blinkStart >= blinkDuration) {
    ledState = !ledState;
    blinkStart = millis();
  }
}

void turn() {
  if (m1Speed < 0) {
    blink(leftLedState, blinkStartLeft);
    rightLedState = HIGH;
  }
  if (m2Speed < 0) {
    blink(rightLedState, blinkStartRight);
    leftLedState = HIGH;
  }
}

void getSensorsValues() {
  for (int i = 0; i < 6; ++i) {
    EEPROM.get(sensorAddress[i], sensorValues);
  }
}

void putSensorsValues() {
  for (int i = 0; i < 6; ++i) {
    EEPROM.put(sensorAddress[i], sensorValues);
  }
}

void debounce() {
  reading = digitalRead(buttonPin);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    buttonState = reading;

    if (buttonState == LOW && !buttonPressed) {
      customCalibrate();
      buttonPressed = true;
    }
  }
  lastReading = reading;
}