#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;


// increase kp’s value and see what happens
float kp = 15;
float ki = 0;
float kd = 0;



float p = 1;
float i = 0;
float d = 0;

float error = 0;
float lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 110;

const int calibrateSpeed = 50;
const unsigned long calibrateTime = 250;
const int calibrateMoves = 24;
QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };
void setup() {

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  // delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode
  // Serial.println("Before custom");
  // setMotorSpeed(100, 0);
  customCalibrate();

  digitalWrite(LED_BUILTIN, LOW);
  setMotorSpeed(0, 0);
}

void loop() {
  // inefficient code, written in loop. You must create separate functions
  float error = map(qtr.readLineBlack(sensorValues), 0, 5000, -20, 20);

  p = error;
  i = i + error;
  d = error - lastError;

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;
  
  pidControl(error);
  
  int motorSpeed = kp * p + ki * i + kd * d;  // = error in this case


  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }
  // make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed properly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making a steep turn
  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);


  setMotorSpeed(m1Speed, m2Speed);
  // debugging();
}

// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse
  //  motor1Speed = -motor1Speed;
  //  motor2Speed = -motor2Speed;
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

  // delay(250);
}

void customCalibrate() {
  // calibrate the sensor. For maximum grade the line follower should do the movement itself, without human interaction.
  int ct = 0;
  int state = 0;
  while (ct <= 6) {
    qtr.calibrate();
    qtr.read(sensorValues);
    if (state == 0) {   // going right
      if (sensorValues[0] < 700) {
        setMotorSpeed(70, -70);
      } else {
        state = 1;
      }
    } if (state == 1) {   // going left
      if (sensorValues[5] < 700) {
        setMotorSpeed(-70, 70);
      } else {
        state = 0;
        ct++;
      }
    }
  }
}

void pidControl(float error) {
  if (error >= -20 && error < -10) {
    kd = 10;
  }
  if (error >= -10 && error < -1) {
    kd = 7;
  }
  if (error >= -1 && error < 0) {
    kd = 2;
  }
  if (error >= 0 && error < 1) {
    kd = -2;
  }   
  if (error >= 1 && error < 10) {
    kd = -7;
  }
  if (error >= 10 && error < 20) {
    kd = -10;
  }

  ki = 0.0045;  
}



