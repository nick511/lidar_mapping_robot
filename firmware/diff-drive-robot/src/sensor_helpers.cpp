#include "sensor_helpers.h"

#include <Wire.h>
#include <ICM_20948.h>

#define AD0_VAL 0 // Set to 1 if AD0 pin is HIGH, 0 if LOW

void imuSetup(ICM_20948_I2C *imuP)
{
  ICM_20948_I2C &imu = *imuP;

  bool initialized = false;

  while (!initialized)
  {
    imu.begin(Wire, AD0_VAL);

    if (imu.status == ICM_20948_Stat_Ok)
    {
      initialized = true;
      Serial.println("IMU connected");
    }
    else
    {
      Serial.println("IMU not detected");
      Serial.println(imu.statusString());
      delay(500);
    }
  }
}

float imuGetGyroZBias(ICM_20948_I2C *imuP)
{
  ICM_20948_I2C &imu = *imuP;

  float sum = 0.0f;
  int samples = 200;
  for (int i = 0; i < samples; i++)
  {
    imu.getAGMT();
    sum += imu.gyrZ() * DEG_TO_RAD; // Ensure units match your loop
    delay(5);
  }

  float gyro_z_bias = sum / (float)samples;

  Serial.print("Calculated Bias: ");
  Serial.println(gyro_z_bias, 6);

  return gyro_z_bias;
}
