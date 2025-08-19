# qcr_force_sensor_ros

ROS driver package for QCR force sensor.

## How to install

- Install dependency of `qcr_force_sensor_ros` (e.g., by `rosdep install`) and build `qcr_force_sensor_ros` (e.g., by `catkin build`)
  - See http://wiki.ros.org/rosdep#Install_dependency_of_all_packages_in_the_workspace for `rosdep` usage

  For example:
  ```bash
  mkdir -p ~/ws_qcr_force_sensor_ros/src
  cd ~/ws_qcr_force_sensor_ros/src
  git clone https://github.com/pazeshun/jsk_3rdparty.git -b qcr_force_sensor_ros
  rosdep install -y -r --ignore-src --from-paths jsk_3rdparty/qcr_force_sensor_ros
  cd ..
  catkin build qcr_force_sensor_ros
  source ~/ws_qcr_force_sensor_ros/devel/setup.bash  # Do this every time you open a new terminal
  ```

## How to use

```bash
# First, plug in your sensor and give permission to its device file (e.g., "sudo chmod a+rw /dev/ttyUSB0").
# Then...
roslaunch qcr_force_sensor_ros sample.launch port:=/dev/ttyUSB0
# On another terminal
rosservice call /qcr_force_sensor_driver/calibrate_offset "{}"  # Do not touch or move the sensor until this command finishes. Call this service every time you want to remove sensor offset
```
