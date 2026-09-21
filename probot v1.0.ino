#define PROBOT_WIFI_AP_PASSWORD "ProBot1234"
#define PROBOT_WIFI_AP_SSID "ITOBOT"
#define PROBOT_WIFI_AP_CHANNEL 9
#include <probot.h>
#include <probot/io/joystick_api.hpp>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// --- AYLAR ---
const int PWM_FREQ = 50;
const int STOP_TICK = 307;
const int Intake_stop_tick = 363;
const int MAX_FWD = 410;
const int MAX_REV = 205;

// Kanal tanimlari
const int M_FL = 0, M_FR = 1, M_RL = 2, M_RR = 3;
const int LIFT = 7, CLIMB1 = 14, CLIMB2 = 15, SERVO = 8;
const int INTAKE = 4;

// Servo kontrol degiskenleri
float servoCurrentPos = 0.0f;
float servoTargetPos = 0.0f;
const float SERVO_SPEED = 8.0f;

// Gyro degiskenleri
float currentYaw = 0.0f;
bool gyroReady = false;
unsigned long lastTelemetryMs = 0;
unsigned long lastGyroUpdateMs = 0;
unsigned long gyroUpdateCount = 0;
uint8_t lastGyroSensorId = 0;

// Otonom degiskenleri
int autoStep = 0;
unsigned long autoTimer = 0;


const float AUTO_FWD_POWER = 0.7f;
const float AUTO_TURN_POWER = 0.9f;  // saga donus icin
const float AUTO_TURN_TOLERANCE = 8.0f;
const unsigned long AUTO_FWD_MS = 1200;
const unsigned long AUTO_STOP_MS = 300;
const unsigned long AUTO_TURN_MS = 600;
const unsigned long AUTO_LIFT_MS = 2000;


// Standart surucu yazma (Spark/Talon)
void writeSpark(int channel, float power) {
  if (fabs(power) < 0.12f) {
    pwm.setPWM(channel, 0, STOP_TICK);
    return;
  }
  int pulse = STOP_TICK + (power * 102);
  pulse = constrain(pulse, MAX_REV, MAX_FWD);
  pwm.setPWM(channel, 0, pulse);
}

void writeIntake(float power) {
  if (fabs(power) < 0.12f) {
    pwm.setPWM(INTAKE, 0, Intake_stop_tick);
    return;
  }

  int pulse = Intake_stop_tick + (power * 102);
  pulse = constrain(pulse, MAX_REV, MAX_FWD);
  pwm.setPWM(INTAKE, 0, pulse);
}

// BTS7960 ozel yazma
void setSingleMotorI2C(int ch, float power) {
  if (power < 0.05f) {
    pwm.setPWM(ch, 0, 0);
    return;
  }
  int pwmVal = (int)(constrain(power, 0, 1.0f) * 4095.0f);
  pwm.setPWM(ch, 0, pwmVal);
}

void writeServoDeg(int channel, int degree) {
  degree = constrain(degree, 0, 270);
  int pulse = map(degree, 0, 270, 102, 614);
  pwm.setPWM(channel, 0, pulse);
}

void writeServo(int channel, float pos) {
  pos = constrain(pos, -1.0f, 1.0f);
  int pulse = STOP_TICK + (pos * 103);
  pulse = constrain(pulse, MAX_REV, MAX_FWD);
  pwm.setPWM(channel, 0, pulse);
}

void updateServoMotion() {
  if (fabs(servoCurrentPos - servoTargetPos) > 0.5f) {
    if (servoCurrentPos < servoTargetPos) {
      servoCurrentPos += SERVO_SPEED;
      if (servoCurrentPos > servoTargetPos) servoCurrentPos = servoTargetPos;
    } else {
      servoCurrentPos -= SERVO_SPEED;
      if (servoCurrentPos < servoTargetPos) servoCurrentPos = servoTargetPos;
    }
    writeServoDeg(SERVO, (int)servoCurrentPos);
  }
}

void updateGyro() {
  if (!gyroReady) return;

  if (bno08x.wasReset()) {
    Serial.println("BNO08x reset!");
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    lastGyroSensorId = sensorValue.sensorId;
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      float qw = sensorValue.un.arvrStabilizedRV.real;
      float qx = sensorValue.un.arvrStabilizedRV.i;
      float qy = sensorValue.un.arvrStabilizedRV.j;
      float qz = sensorValue.un.arvrStabilizedRV.k;

      currentYaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
      currentYaw = currentYaw * 180.0f / PI;
      lastGyroUpdateMs = millis();
      gyroUpdateCount++;
    }
  }
}

void publishTelemetry() {
  if (millis() - lastTelemetryMs < 200) return;

  updateGyro();
  char telemetryBuffer[96];
  unsigned long gyroAgeMs = millis() - lastGyroUpdateMs;
  snprintf(
    telemetryBuffer,
    sizeof(telemetryBuffer),
    "Gyro yaw: %.2f ready:%d age:%lu count:%lu id:%u",
    currentYaw,
    gyroReady ? 1 : 0,
    gyroAgeMs,
    gyroUpdateCount,
    lastGyroSensorId);
  probot::telemetry::println(telemetryBuffer);

  lastTelemetryMs = millis();
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float maxAbs4(float a, float b, float c, float d) {
  float m = fabs(a);
  if (fabs(b) > m) m = fabs(b);
  if (fabs(c) > m) m = fabs(c);
  if (fabs(d) > m) m = fabs(d);
  return m;
}

void stopMotors() {
  writeSpark(M_FL, 0);
  writeSpark(M_FR, 0);
  writeSpark(M_RL, 0);
  writeSpark(M_RR, 0);
}

void driveStraight(float power, float targetYaw) {
  updateGyro();
  float error = normalizeAngle(targetYaw - currentYaw);
  float correction = error * 0.02f;
  correction = constrain(correction, -0.3f, 0.3f);

  float leftPower = power - correction;
  float rightPower = power + correction;

  writeSpark(M_FL, -leftPower);  // FIXED: Inverted sign
  writeSpark(M_RL, leftPower);
  writeSpark(M_FR, rightPower);  // FIXED: Inverted sign (was -rightPower)
  writeSpark(M_RR, -rightPower);
}

void turnInPlace(float turnPower) {
  writeSpark(M_FL, turnPower);  // positive = sag donus
  writeSpark(M_FR, turnPower);
  writeSpark(M_RL, turnPower);
  writeSpark(M_RR, turnPower);
}

void driveManualArcade(float forwardPower, float turnPower) {
  float left = constrain(forwardPower + turnPower, -1.0f, 1.0f);
  float right = constrain(forwardPower - turnPower, -1.0f, 1.0f);

  writeSpark(M_FL, -left);  // FIXED: Inverted sign
  writeSpark(M_RL, left);
  writeSpark(M_FR, right);  // FIXED: Inverted sign (was -right)
  writeSpark(M_RR, -right);
}

void robotInit() {
  Serial.begin(115200);

  Wire.begin(8, 9);
  Wire.setClock(400000);

  if (!bno08x.begin_I2C(0x4A, &Wire) && !bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("BNO08x bulunamadi!");
    gyroReady = false;
  } else {
    Serial.println("BNO08x bulundu!");
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV)) {
      Serial.println("BNO08x report baslatilamadi!");
    }
    gyroReady = true;
    delay(100);
    updateGyro();
  }

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(300);
  pwm.setPWM(CLIMB1, 0, 0);
  pwm.setPWM(CLIMB2, 0, 0);

  servoCurrentPos = 0.0f;
  servoTargetPos = 0.0f;
  writeServoDeg(SERVO, (int)servoCurrentPos);
  delay(500);
}

void teleopInit() {
  stopMotors();
  writeIntake(0.0f);
}

void autonomousInit() {
  stopMotors();
  writeIntake(0.0f);
  writeSpark(LIFT, 0.0f);
  autoStep = 0;
  autoTimer = millis();
  servoTargetPos = 0.0f;
  servoCurrentPos = 0.0f;
  writeServoDeg(SERVO, (int)servoCurrentPos);
}

void autonomousLoop() {
  switch (autoStep) {
    case 0:
      stopMotors();
      autoTimer = millis();
      autoStep = 1;
      break;

    case 1:
      if (millis() - autoTimer < 500) {
        driveManualArcade(AUTO_FWD_POWER, 0.0f);
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 2;
      }
      break;

    case 2:
      if (millis() - autoTimer < AUTO_STOP_MS) {
        stopMotors();
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 3;
      }
      break;

    case 3:
      if (millis() - autoTimer < AUTO_TURN_MS) {
        turnInPlace(AUTO_TURN_POWER); // Mirror: Sola donus (- deger)
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 4;
      }
      break;

    case 4:
      if (millis() - autoTimer < AUTO_FWD_MS) {
        driveManualArcade(AUTO_FWD_POWER, 0.0f);
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 5;
      }
      break;

    default:
      stopMotors();
      break;
  }
}

void teleopLoop() {
  publishTelemetry();
  auto js = probot::io::joystick_api::makeDefault();
  float x = -js.getRawAxis(2);
  float y = -js.getRawAxis(1);
  float z = js.getRawAxis(0);

  float tFL = y + x + z;
  float tFR = y - x - z;
  float tRL = y - x + z;
  float tRR = y + x - z;

  float maxP = max(1.0f, maxAbs4(tFL, tFR, tRL, tRR));

  writeSpark(M_FL, -(tFL / maxP));  // FIXED: Inverted sign
  writeSpark(M_FR, (tFR / maxP));   // FIXED: Inverted sign (was negative, now positive)
  writeSpark(M_RL, tRL / maxP);
  writeSpark(M_RR, -(tRR / maxP));

  if (js.getRawButton(6)) {
    writeIntake(-1.0f);
  } else if (js.getRawButton(10)) {
    writeIntake(1.0f);
  } else {
    writeIntake(0.0f);
  }

  if (js.getRawButton(7)) writeSpark(LIFT, 0.9f);
  else if (js.getRawButton(0)) writeSpark(LIFT, -0.9f);
  else writeSpark(LIFT, 0);

  if (js.getRawButton(12)) {
    pwm.setPWM(CLIMB1, 0, MAX_FWD);
    pwm.setPWM(CLIMB2, 0,MAX_FWD);
  } else if (js.getRawButton(13)) {
    pwm.setPWM(CLIMB1, 0, MAX_REV);
    pwm.setPWM(CLIMB2, 0, MAX_REV);
  } else {
    pwm.setPWM(CLIMB1, 0, 0);
    pwm.setPWM(CLIMB2, 0, 0);
  }

  if (js.getRawButton(4)) {
    servoTargetPos = 200.0f;
  } else if (js.getRawButton(5)) {
    servoTargetPos = -10.0f;
  }
  updateServoMotion();
}

void robotEnd() {}

/*
// --- AYNALANMIS OTONOM (SAHADAN SAGA DOGRU DEGIL, SOLA DOGRU HAREKET ICIN) ---
// Bu kod mevcut otonomun tam tersi yonunde (sola) donusler yapar.
// Aktiflestirmek isterseniz autonomousLoop() icerigini bununla degistirebilirsiniz.

void autonomousLoopMirrored() {
  switch (autoStep) {
    case 0:
      stopMotors();
      autoTimer = millis();
      autoStep = 1;
      break;

    case 1:
      if (millis() - autoTimer < 500) {
        driveManualArcade(AUTO_FWD_POWER, 0.0f);
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 2;
      }
      break;

    case 2:
      if (millis() - autoTimer < AUTO_STOP_MS) {
        stopMotors();
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 3;
      }
      break;

    case 3:
      if (millis() - autoTimer < AUTO_TURN_MS) {
        turnInPlace(AUTO_TURN_POWER); // Mirror: Sola donus (- deger)
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 4;
      }
      break;

    case 4:
      if (millis() - autoTimer < AUTO_FWD_MS) {
        driveManualArcade(AUTO_FWD_POWER, 0.0f);
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 5;
      }
      break;

    case 5:
      if (millis() - autoTimer < AUTO_STOP_MS) {
        stopMotors();
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 6;
      }
      break;

    case 6:
      if (millis() - autoTimer < AUTO_TURN_MS) {
        turnInPlace(-AUTO_TURN_POWER); // Mirror: Sola donus (- deger)
      } else {
        stopMotors();
        autoTimer = millis();
        autoStep = 7;
      }
      break;

    case 7:
      servoTargetPos = 200.0f;
      updateServoMotion();
      if (millis() - autoTimer < AUTO_FWD_MS) {
        driveManualArcade(AUTO_FWD_POWER, 0.0f);
      } else {
        stopMotors();
        writeSpark(LIFT, 0.9f);
        autoTimer = millis();
        autoStep = 8;
      }
      break;

    case 8:
      servoTargetPos = 200.0f;
      updateServoMotion();
      stopMotors();
      writeSpark(LIFT, 0.9f);
      if (millis() - autoTimer >= AUTO_LIFT_MS) {
        writeSpark(LIFT, 0.0f);
        autoStep = 9;
      }
      break;

    case 9:
      servoTargetPos = 200.0f;
      updateServoMotion();
      stopMotors();
      writeSpark(LIFT, 0.0f);
      break;

    default:
      stopMotors();
      break;
  }
}
*/
