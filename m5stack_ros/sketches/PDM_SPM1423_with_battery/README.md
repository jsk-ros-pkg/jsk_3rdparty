# PDM SPM1423 with battery

This firmware is an addition to the [PDM_SPM1423](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/PDM_SPM1423) firmware with battery monitoring functionality.

In `PDM_SPM1423_with_battery` version, battery infomation (`battery_level` and `is_charging` rostopic) are published in addition to audio information.

## Note
  - If you connect PDM_SPM1423 microphone to M5Stack Grove connector, you cannot use I2S and I2C devices simultaneously. In that case, try `enableI2C` and `disableI2C` functions defined in [PDM_SPM1423.h](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/arduino_libraries/PDM_SPM1423.h).
  - The sample program which uses I2S microphone and I2C battery management module is under [PDM_SPM1423_with_battery](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros/sketches/PDM_SPM1423/PDM_SPM1423_with_battery)
    - When acquiring battery information, the connection to the microphone temporarily stops.
    - Therefore, there are times when the microphone information is temporarily unavailable
