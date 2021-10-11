// See https://github.com/tom2rd/ESP32room/blob/master/MPU9250/IMU.ino

// define must ahead #include <M5Stack.h>
// #define M5STACK_MPU6886
 #define M5STACK_MPU9250
// #define M5STACK_MPU6050
// #define M5STACK_200Q

#include <M5Stack.h>

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float qx    = 0.0F;
float qy    = 0.0F;
float qz    = 0.0F;
float qw    = 0.0F;

float temp = 0.0F;

void setupMPU9250()
{
  M5.Power.begin();
  M5.IMU.Init();
}

void measureMPU9250()
{
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
  M5.IMU.getTempData(&temp);
}

float deg2rad(float a)
{
  return a / 180.0 * M_PI;
}

// Copied from https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
// rosserial does not have function to create quaternion from roll, pitch and yaw.
// https://answers.ros.org/question/337231/create-quaternion-from-euler-in-arduino/
// https://github.com/ros-drivers/rosserial/tree/noetic-devel/rosserial_client/src/ros_lib/tf
void EulerAnglesToQuaternion(float roll, float pitch, float yaw,
                            float& q0, float& q1, float& q2, float& q3)
{
    float cosRoll = cos(roll / 2.0);
    float sinRoll = sin(roll / 2.0);
    float cosPitch = cos(pitch / 2.0);
    float sinPitch = sin(pitch / 2.0);
    float cosYaw = cos(yaw / 2.0);
    float sinYaw = sin(yaw / 2.0);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; // qw
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; // qx
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; // qy
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; // qz
}

void displayMPU9250() {
  // M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.setCursor(0, 10);
  M5.Lcd.printf("Gyro [o/s]\n", gyroX, gyroY, gyroZ);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(0, 65);
  M5.Lcd.print("Accel [G]\n");
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.print("AHRS\n");
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);
  M5.Lcd.setCursor(0, 175);
  M5.Lcd.print("Temp [C]\n");
  M5.Lcd.printf("%.2f", temp);
}
