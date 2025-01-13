#include <limits.h>

#include <ros.h>
#include <bl_force_torque_sensor_ros/AmplifierOutputRawArray.h>
#include <std_msgs/Float32.h>

#include "ADS131M04.h"

#define ADC_NUM 2
#define VALID_CH_NUM 3
ADS131M04 adc[ADC_NUM];
adcOutput adc_res[ADC_NUM];

unsigned long loop_duration = 0;  // loop duration in us, set via rostopic callback
unsigned long start_time;

ros::NodeHandle nh;
bl_force_torque_sensor_ros::AmplifierOutputRawArray output_msg;
ros::Publisher output_pub("~output", &output_msg);
int32_t output_array[ADC_NUM * VALID_CH_NUM];

void durationCb(const std_msgs::Float32& duration_msg)
{
  loop_duration = duration_msg.data * 1000 * 1000;  // sec -> us
}

ros::Subscriber<std_msgs::Float32> duration_sub("~parameter_getter/loop_duration", &durationCb);

void setup()
{
  nh.getHardware()->setBaud(250000);
  // cf. https://arduino.stackexchange.com/questions/296/how-high-of-a-baud-rate-can-i-go-without-errors
  //     http://wormfood.net/avrbaudcalc.php
  // I'm not sure why, but 500000 causes "Mismatched protocol version" error
  nh.initNode();
  nh.advertise(output_pub);
  nh.subscribe(duration_sub);
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  adc[0].begin(9, 7, 4000000);
  adc[1].begin(8, 6, 4000000);  // I'm not sure why, but using D10 as CS doesn't work. Conflicted with SPI.begin()?
}

void loop()
{
  start_time = micros();

  // Wait until data is ready
  bool is_ok = false;
  while (!is_ok)
  {
    is_ok = true;
    for (int i = 0; i < ADC_NUM; i++)
    {
      if (!adc[i].isDataReady())
      {
        is_ok = false;
      }
    }
  }

  is_ok = true;
  for (int i = 0; i < ADC_NUM; i++)
  {
    if (!adc[i].readADC(&adc_res[i]))
    {
      is_ok = false;
    }
    delayMicroseconds(1);  // Should be longer than t_w(CSH)?
  }
  for (int i = 0; i < ADC_NUM; i++)
  {
    if (adc_res[i].status != 0x050F)
    {
      // status is value of STATUS register.
      // By default, if everything is OK, this value is 0000 0101 0000 1111 (0x050F)
      is_ok = false;
    }
  }
  if (is_ok)
  {
    for (int i = 0; i < ADC_NUM; i++)
    {
      output_array[i * VALID_CH_NUM] = adc_res[i].ch0;
      output_array[i * VALID_CH_NUM + 1] = adc_res[i].ch1;
      output_array[i * VALID_CH_NUM + 2] = adc_res[i].ch2;
    }
    output_msg.array = output_array;
    output_msg.array_length = ADC_NUM * VALID_CH_NUM;
    output_pub.publish(&output_msg);
  }

  nh.spinOnce();
  // Enforce constant loop time
  if ((ULONG_MAX - start_time) < loop_duration)
  {
    // micros() will overflow while waiting
    while (micros() >= start_time || micros() < (loop_duration - (ULONG_MAX - start_time)));
  }
  else if (micros() < start_time);  // micros() already overflowed unexpectedly
  else
  {
    // Normal waiting
    while ((micros() - start_time) < loop_duration);
  }
}
