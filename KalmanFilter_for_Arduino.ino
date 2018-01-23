#include <Wire.h>
#include "Kalman.h"
#include "MPU6050.h"

// 加速度/ジャイロセンサーの制御変数。
KalmanFilter gKfx, gKfy; // カルマンフィルタ。
MPU6050 mpu(true);

float gCalibrateX; // 初期化時の角度。（＝静止角とみなす）
float gCalibrateY;
long gPrevMicros; // loop()間隔の計測用。

void setup() {
  Serial.begin(115200);
  mpu.begin();
  int16_t acc[3],gyro[3];
  mpu.get_data(acc, gyro); 
  float ax = (float)acc[0];
  float ay = (float)acc[1];
  float az = (float)acc[2];
  float degRoll  = atan2(ay, az) * RAD_TO_DEG;
  float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  gKfx.setAngle(degRoll);
  gKfy.setAngle(degPitch);
  gCalibrateY = degPitch;
  gCalibrateX = degRoll;
  gPrevMicros = micros();
}

void loop() {
  int16_t acc[3],gyro[3];
  // 重力加速度から角度を求める。
  mpu.get_data(acc, gyro); 
  float ax = (float)acc[0];
  float ay = (float)acc[1];
  float az = (float)acc[2];
  float degRoll  = atan2(ay, az) * RAD_TO_DEG;
  float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // ジャイロで角速度を求める。
  float dpsX = (float)gyro[0] / 131.0;
  float dpsY = (float)gyro[1] / 131.0;
  float dpsZ = (float)gyro[2] / 131.0;

  // カルマンフィルタで角度(x,y)を計算する。
  long curMicros = micros();
  float dt = (float)(curMicros - gPrevMicros) / 1000000; // μsec -> sec
  gPrevMicros = curMicros;
  float degX = gKfx.calcAngle(degRoll, dpsX, dt);
  float degY = gKfy.calcAngle(degPitch, dpsY, dt);
  degY -= gCalibrateY;
  degX -= gCalibrateX;

  // シリアル送信
  static int ps;
  if (++ps % 1 == 0) {
    Serial.print(dt,5);
    Serial.print(",");
    Serial.print(degY);
    Serial.print(",");
    Serial.print(degX);
    Serial.println("");
  }
  delay(1);
}
