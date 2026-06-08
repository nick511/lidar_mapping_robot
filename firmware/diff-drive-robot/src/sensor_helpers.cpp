#include "sensor_helpers.h"

#include <Wire.h>
#include <ICM_20948.h>

#define AD0_VAL 0 // Set to 1 if AD0 pin is HIGH, 0 if LOW

void scanI2C()
{
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

float gyro_z_bias = 0.0f;
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
      scanI2C();
      delay(500);
    }
  }

  gyro_z_bias = imuGetGyroZBias(&imu);
}

float imuGetGyroZBias(ICM_20948_I2C *imuP)
{
  ICM_20948_I2C &imu = *imuP;

  Serial.println("Calculating gyro Z Bias...");

  float sum = 0.0f;
  int samples = 200;
  for (int i = 0; i < samples; i++)
  {
    imu.getAGMT();
    sum += imu.gyrZ();
    delay(5);
  }

  float gyro_z_bias = sum / samples;

  Serial.print("Calculated Bias: ");
  Serial.println(gyro_z_bias, 6);

  return gyro_z_bias;
}

const float GYRO_Z_DEADBAND = 0.3f;
float gyro_z_filtered = 0.0f;
float getCleanGyroZ(ICM_20948_I2C *imuP)
{
  ICM_20948_I2C &imu = *imuP;
  float gyro_z = imu.gyrZ() - gyro_z_bias; // Remove bias from raw gyro Z reading

  if (gyro_z >= -GYRO_Z_DEADBAND && gyro_z <= GYRO_Z_DEADBAND)
  {
    return 0.0f; // Treat small values as zero to reduce noise
  }

  gyro_z_filtered = 0.9 * gyro_z_filtered + 0.1 * gyro_z; // Simple low-pass filter to reduce noise

  return gyro_z_filtered;
}
