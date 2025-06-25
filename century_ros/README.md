# century_ros

ROS tools for [Century products](https://www.century.co.jp/products/).

## [USB-Serial troubleshooter](https://www.century.co.jp/products/ct-3usb1hub.html)

### Preparation

#### Udev setting

This is optional, but we recommend this setting because USB-Serial troubleshooter is sometimes suddenly disconnected and its device file changes.
```bash
rosrun century_ros setup_udev_for_usb_serial_troubleshooter <serial_number> <symlink_name>
# <serial_number> : The serial number of your USB-Serial troubleshooter (e.g., 001A02F2046C). You can check the serial number by "udevadm info --name=/dev/ttyACM* --attribute-walk"
# <symlink_name> : The name to create as a symbolic link in /dev (e.g., usb_serial_troubleshooter1)
```

#### Connecting USB-Serial troubleshooter

When inserting your USB-Serial troubleshooter to your PC, check if LED in the troubleshooter becomes blue after a few moments.
If it remains red, the connection between the troubleshooter and the PC fails to be established.
If you face this phenomenon, try changing the insertion process.
Our experience has shown that success is more likely to be achieved if the insertion is stopped at a shallow point and then immediately deepened.

### Minimal usage: launching driver only

```bash
roslaunch century_ros usb_serial_troubleshooter.launch port:=/dev/<symlink_name>
# <symlink_name> : The same as above
```

#### Arguments

Check them by `roslaunch century_ros usb_serial_troubleshooter.launch --ros-args`:
```
Required Arguments:
  port: Port connecting with USB-Serial troubleshooter (e.g., '/dev/ttyACM0')
Optional Arguments:
  init_with_power_on (default "true"): Power on USB when driver of USB-Serial troubleshooter is initialized
  power_cycle_interval (default "1.0"): Interval [sec] in USB power cycle using USB-Serial troubleshooter
  serial_timeout (default "1"): Timeout [sec] of serial communication with USB-Serial troubleshooter
```

#### Services

- `usb_serial_troubleshooter_driver/power` (`std_srvs/SetBool`)

  Power on/off USB.

- `usb_serial_troubleshooter_driver/power_cycle` (`std_srvs/Trigger`)

  Execute power cycle of USB.

### Advanced usage: automated power cycle of USB

```bash
roslaunch century_ros usb_auto_power_cycle.launch troubleshooter_port:=/dev/<symlink_name> monitored_topic:=<monitored_topic> monitored_topic_expected_hz:=<monitored_topic_expected_hz> monitored_topic_respawn_delay:=<monitored_topic_respawn_delay>
# <symlink_name> : The same as above
# <monitored_topic> : Topic monitored for determining USB health. If this is not published for an unusually long period, USB power cycle occurs
# <monitored_topic_expected_hz> : Expected Hz of monitored topic
# <monitored_topic_respawn_delay> : For this duration [sec] after USB power cycle, new power cycle does not occur to wait for topic to be published
```
