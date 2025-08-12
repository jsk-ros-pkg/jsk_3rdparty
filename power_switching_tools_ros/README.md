# power_switching_tools_ros

ROS tools for power switching tools.  
Supported tools are:
- [USB-Serial troubleshooter](#usb-serial-troubleshooter)
- [USB PPPS hubs](#usb-ppps-hubs)

## [USB-Serial troubleshooter](https://www.century.co.jp/products/ct-3usb1hub.html)

### Preparation

#### Udev setting

This is optional, but we recommend this setting because USB-Serial troubleshooter is sometimes suddenly disconnected and its device file changes.
```bash
rosrun power_switching_tools_ros setup_udev_for_usb_serial_troubleshooter <serial_number> <symlink_name>
# <serial_number> : The serial number of your USB-Serial troubleshooter (e.g., 001A02F2046C). You can check the serial number by "udevadm info --name=/dev/ttyACM* --attribute-walk"
# <symlink_name> : The name to create as a symbolic link in /dev (e.g., usb_serial_troubleshooter1)
```

#### Connecting USB-Serial troubleshooter

When inserting your USB-Serial troubleshooter to your PC, check if LED in the troubleshooter becomes blue after a few moments.
If it remains red, the connection between the troubleshooter and the PC fails to be established.
If you face this phenomenon, try the following:
- Reversing USB-C connector of USB hub/adapter/cable if you connect the troubleshooter to a USB hub/adapter/cable connected to a USB-C port of the PC
- Connecting the troubleshooter to a USB-A port of the PC directly (not via a USB hub)
- Connecting the troubleshooter to a USB-C port of the PC via an adapter/cable ([example](https://www.amazon.co.jp/dp/B09SFS9C5K))
- Changing USB hub/adapter/cable
- Changing the insertion process. Our experience has shown that success is more likely to be achieved if the insertion is stopped at a shallow point and then immediately deepened
- Using USB 2.0 hub/adapter/cable. Note that you will be unable to use USB 3.0 features on the devices under the troubleshooter

### Minimal usage: launching driver only

```bash
roslaunch power_switching_tools_ros usb_serial_troubleshooter.launch port:=/dev/<symlink_name>
# <symlink_name> : The same as above
```

#### Arguments

Check them by `roslaunch power_switching_tools_ros usb_serial_troubleshooter.launch --ros-args`:
```
Required Arguments:
  port: Port connecting with USB-Serial troubleshooter (e.g., '/dev/ttyACM0')
Optional Arguments:
  init_with_power_on (default "true"): Power on USB when driver of USB-Serial troubleshooter is initialized. This argument is valid only when init_with_power_set is true
  init_with_power_set (default "true"): Set power state of USB when driver of USB-Serial troubleshooter is initialized. Which state (ON or OFF) is set is defined by init_with_power_on
  power_cycle_interval (default "1.0"): Interval [sec] in USB power cycle using USB-Serial troubleshooter
  power_cycle_service (default "~power_cycle"): Name of service for USB power cycle
  power_service (default "~power"): Name of service for USB power switching
  serial_timeout (default "1"): Timeout [sec] of serial communication with USB-Serial troubleshooter
```

#### Services

- `usb_serial_troubleshooter_driver/power` (`std_srvs/SetBool`)

  Power on/off USB.

- `usb_serial_troubleshooter_driver/power_cycle` (`std_srvs/Trigger`)

  Execute power cycle of USB.

### Advanced usage: automated power cycle of USB

```bash
roslaunch power_switching_tools_ros auto_power_cycle.launch troubleshooter_port:=/dev/<symlink_name> monitored_topic:=<monitored_topic> monitored_topic_expected_hz:=<monitored_topic_expected_hz> monitored_topic_respawn_delay:=<monitored_topic_respawn_delay>
# <symlink_name> : The same as above
# <monitored_topic> : Topic monitored for determining health of monitored communication. If this is not published for an unusually long period, power cycle of that communication occurs
# <monitored_topic_expected_hz> : Expected Hz of monitored topic
# <monitored_topic_respawn_delay> : For this duration [sec] after power cycle, new power cycle does not occur to wait for topic to be published
#
# In default, this launch firstly executes power cycle once. If you want to change this behavior, set "init_with_power_cycle:=false"
```

## USB PPPS hubs

"PPPS" stands for "Per-Port Power Switching" which is an advanced functionality of USB hubs.
Hubs supporting PPPS (e.g., [VirtualHere USB 3 4-Port Hub](https://modularkvmip.com/product/usb-hub/)) are listed [here](https://github.com/mvp/uhubctl?tab=readme-ov-file#compatible-usb-hubs).
- Advantages compared with USB-Serial troubleshooter:
  - Having multiple ports
  - [Connection problem](#connecting-usb-serial-troubleshooter) does not occur
- Disadvantages compared with USB-Serial troubleshooter:
  - Power switching is slower
  - Some devices are hard to be powered off (e.g., [arbitrary power recovering problem](https://github.com/mvp/uhubctl?tab=readme-ov-file#power-comes-back-on-after-few-seconds-on-linux))

### Preparation

#### Udev setting

This is required for running the driver node without `sudo`.
```bash
rosrun power_switching_tools_ros setup_udev_for_usb_ppps_hub
# Reboot your PC after this script finishes
# Note that this script adds your user to dialout group, which means "chmod" becomes unnecessary for accessing some device files (e.g., /dev/ttyACM0)
```

#### Check if power switching works

Connect your hub to your PC, connect devices you want to use in your application to your hub, then run:
```bash
uhubctl  # If this command is not found, install it by "sudo apt install uhubctl"
```
You will get an output like:
```
Current status for hub 3-5 [366b:0004 VirtualHere Pty. Ltd. VirtualHere 4-Port Hub 91B35D672F3E3755, USB 2.10, 4 ports]
  Port 1: 0103 power enable connect [2341:0058 Arduino LLC Arduino Nano Every 059D618F51514C4B38202020FF171A11]
  Port 2: 0100 power
  Port 3: 0100 power
  Port 4: 0100 power
Current status for hub 2-2 [366b:0005 VirtualHere Pty. Ltd. VirtualHere 4-Port Hub 91B35D672F3E3755, USB 3.20, 4 ports]
  Port 1: 06a0 power Rx.Detect
  Port 2: 06a0 power Rx.Detect
  Port 3: 06a0 power Rx.Detect
  Port 4: 06a0 power Rx.Detect
```
Write down the number just after `Current status for hub` whose section includes the device you want to power on/off (`3-5`, in the example above).
This is the location of your hub.
In addition, write down the port which that device is connected to (`1`, in the example above).
Then try to power off and on the device:
```bash
uhubctl -a off -l <hub_location> -p <hub_port>
# <hub_location> : Location of your hub
# <hub_port> : Port of your hub which you want to power on/off
uhubctl -a on -l <hub_location> -p <hub_port>
```
If USB power supply to the device is stopped and restarted as expected, `power_switching_tools_ros` can also power on/off the device.
If not, check [the web page of uhubctl](https://github.com/mvp/uhubctl) (especially [FAQ](https://github.com/mvp/uhubctl?tab=readme-ov-file#faq)) to find candidates for a workaround and try them.
If the found workaround requires supplementary options for `uhubctl`, write them down.

### Minimal usage: launching driver only

```bash
roslaunch power_switching_tools_ros usb_ppps_hub.launch hub_location:=<hub_location> hub_port:=<hub_port>
# <hub_location> : Location of your hub
# <hub_port> : Port of your hub which you want to power on/off
```
If [the previous checking](#check-if-power-switching-works) tells you to add supplementary options to `uhubctl`, use `uhubctl_executable` argument explained below.

#### Arguments

Check them by `roslaunch power_switching_tools_ros usb_ppps_hub.launch --ros-args`:
```
Required Arguments:
  hub_location: Location of USB PPPS hub (e.g., '3-5')
  hub_port: Port of USB PPPS hub you want to power on/off (e.g., '1')
Optional Arguments:
  init_with_power_on (default "true"): Power on USB when driver of USB PPPS hub is initialized. This argument is valid only when init_with_power_set is true
  init_with_power_set (default "true"): Set power state of USB when driver of USB PPPS hub is initialized. Which state (ON or OFF) is set is defined by init_with_power_on
  power_cycle_interval (default "1.0"): Interval [sec] in USB power cycle using USB PPPS hub
  power_cycle_service (default "~power_cycle"): Name of service for USB power cycle
  power_service (default "~power"): Name of service for USB power switching
  uhubctl_executable (default "uhubctl"): Command to control USB PPPS hub. You can add supplementary options to uhubctl command (e.g., 'uhubctl -r 100'). In addition, you can specify the path to uhubctl command you installed on your own (e.g., '~/uhubctl/uhubctl')
```

#### Services

- `usb_ppps_hub_driver/power` (`std_srvs/SetBool`)

  Power on/off the target port.

- `usb_ppps_hub_driver/power_cycle` (`std_srvs/Trigger`)

  Execute power cycle of the target port.

### Advanced usage: automated power cycle of USB hub port

```bash
roslaunch power_switching_tools_ros usb_ppps_hub.launch power_cycle_service:=auto_power_cycle/power_cycle hub_location:=<hub_location> hub_port:=<hub_port>
roslaunch power_switching_tools_ros auto_power_cycle.launch launch_default_driver:=false monitored_topic:=<monitored_topic> monitored_topic_expected_hz:=<monitored_topic_expected_hz> monitored_topic_respawn_delay:=<monitored_topic_respawn_delay>
# <monitored_topic> : Topic monitored for determining health of monitored communication. If this is not published for an unusually long period, power cycle of that communication occurs
# <monitored_topic_expected_hz> : Expected Hz of monitored topic
# <monitored_topic_respawn_delay> : For this duration [sec] after power cycle, new power cycle does not occur to wait for topic to be published
#
# In default, this launch firstly executes power cycle once. If you want to change this behavior, set "init_with_power_cycle:=false"
```
