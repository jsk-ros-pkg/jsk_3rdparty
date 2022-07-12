#include <MPU9250.h>
#include <m5stack_ros_with_battery.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
sensor_msgs::Temperature temp_msg;
ros::Publisher temp_pub("temperature", &temp_msg);

void setup()
{
  setupM5stackROS();
  setupMPU9250();
  nh.advertise(imu_pub);
  nh.advertise(temp_pub);

  afterSetup();
}

void loop()
{
  checkCharge();

  measureMPU9250();
  displayMPU9250();

  // Publish IMU
  EulerAnglesToQuaternion(deg2rad(roll), deg2rad(pitch), deg2rad(yaw), qw, qx, qy, qz);
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;
  imu_msg.angular_velocity.x = gyroX;
  imu_msg.angular_velocity.y = gyroY;
  imu_msg.angular_velocity.z = gyroZ;
  imu_msg.linear_acceleration.x = accX;
  imu_msg.linear_acceleration.y = accY;
  imu_msg.linear_acceleration.z = accZ;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "m5stack";
  imu_pub.publish(&imu_msg);
  // Publish temperature
  temp_msg.temperature = temp;
  temp_msg.header.stamp = nh.now();
  temp_msg.header.frame_id = "m5stack";
  temp_pub.publish(&temp_msg);
  nh.spinOnce();

  delay(100);
}
