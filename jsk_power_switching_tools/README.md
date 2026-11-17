# jsk_power_switching_tools

ROS drivers of USB power switches, to power on/off or power-cycle a USB device (e.g. a camera or a serial device
which hangs) by a service call, and a node which power-cycles it automatically when its topic stops.
Supported tools are:
- [USB-Serial troubleshooter](#usb-serial-troubleshooter)
- [USB PPPS hubs](#usb-ppps-hubs)

| Node | ROS 1 | ROS 2 | |
|---|:---:|:---:|---|
| [`usb_serial_troubleshooter_driver.py`](#usb_serial_troubleshooter_driverpy) | ✓ | ✓ | Driver of USB-Serial troubleshooter |
| [`usb_ppps_hub_driver.py`](#usb_ppps_hub_driverpy) | ✓ | ✓ | Driver of USB PPPS hubs (via `uhubctl`) |
| [`auto_power_cycle.py`](#auto_power_cyclepy) | ✓ | | Power cycle when a monitored topic stops |

The launch files (`launch/*.launch`) are of ROS 1; on ROS 2 run the drivers with `ros2 run`.

## [USB-Serial troubleshooter](https://www.century.co.jp/products/ct-3usb1hub.html)

### Preparation

#### Udev setting

This is optional, but we recommend this setting because USB-Serial troubleshooter is sometimes suddenly disconnected and its device file changes.
```bash
# ROS 1
rosrun jsk_power_switching_tools setup_udev_for_usb_serial_troubleshooter <serial_number> <symlink_name>
# <serial_number> : The serial number of your USB-Serial troubleshooter (e.g., 001A02F2046C). You can check the serial number by "udevadm info --name=/dev/ttyACM* --attribute-walk"
# <symlink_name> : The name to create as a symbolic link in /dev (e.g., usb_serial_troubleshooter1)
```
The script finds the udev rule template by `rospack`, so it does not work on ROS 2. There, write the rule of
`udev/90-usb-serial-troubleshooter.rules.template` by hand:
```bash
sed -e 's/{{SERIAL}}/<serial_number>/' -e 's/{{SYMLINK}}/<symlink_name>/' \
  $(ros2 pkg prefix --share jsk_power_switching_tools)/udev/90-usb-serial-troubleshooter.rules.template \
  | sudo tee /etc/udev/rules.d/90-usb-serial-troubleshooter.rules
sudo udevadm control --reload && sudo udevadm trigger
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

ROS 2:
```bash
ros2 run jsk_power_switching_tools usb_serial_troubleshooter_driver.py --ros-args \
  -p port:=/dev/<symlink_name> -p serial_timeout:=1.0
ros2 service call /usb_serial_troubleshooter_driver/power std_srvs/srv/SetBool '{data: false}'
ros2 service call /usb_serial_troubleshooter_driver/power_cycle std_srvs/srv/Trigger
```

ROS 1:
```bash
roslaunch jsk_power_switching_tools usb_serial_troubleshooter.launch port:=/dev/<symlink_name>
# <symlink_name> : The same as above
rosservice call /usb_serial_troubleshooter_driver/power "data: false"
rosservice call /usb_serial_troubleshooter_driver/power_cycle
```

#### Arguments

Check them by `roslaunch jsk_power_switching_tools usb_serial_troubleshooter.launch --ros-args`:
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

### Advanced usage: automated power cycle of USB (ROS 1)

```bash
roslaunch jsk_power_switching_tools auto_power_cycle.launch troubleshooter_port:=/dev/<symlink_name> monitored_topic:=<monitored_topic> monitored_topic_expected_hz:=<monitored_topic_expected_hz> monitored_topic_respawn_delay:=<monitored_topic_respawn_delay>
# <symlink_name> : The same as above
# <monitored_topic> : Topic monitored for determining health of monitored communication. If this is not published for an unusually long period, power cycle of that communication occurs
# <monitored_topic_expected_hz> : Expected Hz of monitored topic
# <monitored_topic_respawn_delay> : For this duration [sec] after power cycle, new power cycle does not occur to wait for topic to be published
#
# In default, this launch firstly executes power cycle once. If you want to change this behavior, set "init_with_power_cycle:=false"
```

Other arguments of `auto_power_cycle.launch` (check them by `--ros-args`): `launch_default_driver` (default
`true`: run `usb_serial_troubleshooter.launch`), `troubleshooter_serial_timeout` (`1`),
`troubleshooter_power_cycle_interval` (`1.0`), `troubleshooter_init_with_power_set` (`true`),
`troubleshooter_init_with_power_on` (`true`), `monitored_topic_allowed_delay_cycles` (`2.0`) and
`init_with_power_cycle` (`true`).

## USB PPPS hubs

"PPPS" stands for "Per-Port Power Switching" which is an advanced functionality of USB hubs.
Hubs supporting PPPS (e.g., [VirtualHere USB 3 4-Port Hub](https://modularkvmip.com/product/usb-hub/)) are listed [here](https://github.com/mvp/uhubctl?tab=readme-ov-file#compatible-usb-hubs).
- Advantages compared with USB-Serial troubleshooter:
  - Having multiple ports
  - [Connection problem](#connecting-usb-serial-troubleshooter) does not occur
- Disadvantages compared with USB-Serial troubleshooter:
  - Power switching is slower
  - Some devices show unexpected behavior after you command your hub to power off them
    - [Power comes back on after few seconds](https://github.com/mvp/uhubctl?tab=readme-ov-file#power-comes-back-on-after-few-seconds-on-linux)
    - Power comes back on when the device is accessed.
      This behavior was observed when webcams were used on Ubuntu 20.04 (not observed on Ubuntu 24.04).
      This behavior is slightly different from the first behavior because the first behavior depends on no external access.
      The workarounds for the first behavior on Ubuntu 20.04 (i.e., Linux 5.x) mentioned in the document do not work for the second behavior

### Preparation

#### Udev setting

This is required for running the driver node without `sudo`.
```bash
# ROS 1
rosrun jsk_power_switching_tools setup_udev_for_usb_ppps_hub
# ROS 2
$(ros2 pkg prefix --share jsk_power_switching_tools)/scripts/setup_udev_for_usb_ppps_hub
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
If USB power supply to the device is stopped and restarted as expected, `jsk_power_switching_tools` can also power on/off the device.
If not, try the latest version of `uhubctl`:
```bash
cd ~
git clone https://github.com/mvp/uhubctl.git
cd uhubctl
make
./uhubctl -a off -l <hub_location> -p <hub_port>
./uhubctl -a on -l <hub_location> -p <hub_port>
```
If this works as expected, use `~/uhubctl/uhubctl` to control your hub in the following (i.e., pass `uhubctl_executable:=~/uhubctl/uhubctl` to `usb_ppps_hub.launch`).
If not, check [the web page of uhubctl](https://github.com/mvp/uhubctl) (especially [FAQ](https://github.com/mvp/uhubctl?tab=readme-ov-file#faq)) to find candidates for a workaround and try them.
If the found workaround requires supplementary options for `uhubctl`, write them down.

### Minimal usage: launching driver only

ROS 2:
```bash
ros2 run jsk_power_switching_tools usb_ppps_hub_driver.py --ros-args \
  -p hub_location:=<hub_location> -p hub_port:=<hub_port>
# several ports: -p hub_location:="['3-5', '3-5']" -p hub_port:="[1, 2]"
ros2 service call /usb_ppps_hub_driver/power std_srvs/srv/SetBool '{data: false}'
ros2 service call /usb_ppps_hub_driver/power_cycle std_srvs/srv/Trigger
```

ROS 1:
```bash
roslaunch jsk_power_switching_tools usb_ppps_hub.launch hub_location:=<hub_location> hub_port:=<hub_port>
# <hub_location> : Location of your hub
# <hub_port> : Port of your hub which you want to power on/off
```
If [the previous checking](#check-if-power-switching-works) tells you to use the latest `uhubctl` and/or add supplementary options to `uhubctl`, use `uhubctl_executable` argument (parameter on ROS 2) explained below.

#### Arguments

Check them by `roslaunch jsk_power_switching_tools usb_ppps_hub.launch --ros-args`:
```
Required Arguments:
  hub_location: Location(s) of USB PPPS hub (e.g., '3-5', '[3-5, 2-2]', , '[3-5, 3-5]')
  hub_port: Port(s) of USB PPPS hub you want to power on/off (e.g., '1', '[1, 1]', '[1, 2]'). Must be the same length as hub_location and each element corresponds to the element at the same position in hub_location
Optional Arguments:
  init_with_power_on (default "true"): Power on USB when driver of USB PPPS hub is initialized. This argument is valid only when init_with_power_set is true
  init_with_power_set (default "true"): Set power state of USB when driver of USB PPPS hub is initialized. Which state (ON or OFF) is set is defined by init_with_power_on
  power_cycle_interval (default "1.0"): Interval [sec] in USB power cycle using USB PPPS hub
  power_cycle_service (default "~power_cycle"): Name of service for USB power cycle
  power_service (default "~power"): Name of service for USB power switching
  uhubctl_executable (default "uhubctl"): Command to control USB PPPS hub. You can add supplementary options to uhubctl command (e.g., 'uhubctl -r 100'). In addition, you can specify the path to uhubctl command you installed on your own (e.g., '~/uhubctl/uhubctl')
```

### Advanced usage: automated power cycle of USB hub port (ROS 1)

```bash
roslaunch jsk_power_switching_tools usb_ppps_hub.launch power_cycle_service:=auto_power_cycle/power_cycle hub_location:=<hub_location> hub_port:=<hub_port>
roslaunch jsk_power_switching_tools auto_power_cycle.launch launch_default_driver:=false monitored_topic:=<monitored_topic> monitored_topic_expected_hz:=<monitored_topic_expected_hz> monitored_topic_respawn_delay:=<monitored_topic_respawn_delay>
# <monitored_topic> : Topic monitored for determining health of monitored communication. If this is not published for an unusually long period, power cycle of that communication occurs
# <monitored_topic_expected_hz> : Expected Hz of monitored topic
# <monitored_topic_respawn_delay> : For this duration [sec] after power cycle, new power cycle does not occur to wait for topic to be published
#
# In default, this launch firstly executes power cycle once. If you want to change this behavior, set "init_with_power_cycle:=false"
```

## Interface

### Common to the drivers

Services:

- `~power` (`std_srvs/SetBool`): power on (`true`) / off (`false`) the USB device
- `~power_cycle` (`std_srvs/Trigger`): power off, wait `~power_cycle_interval`, power on

Parameters:

| Parameter | Type | Default | |
|---|---|---|---|
| `~power_cycle_interval` | double | `1.0` | Interval [s] between off and on of `~power_cycle` |
| `~init_with_power_set` | bool | `true` | Set the power state at start up |
| `~init_with_power_on` | bool | `true` | The state set at start up: on (`true`) or off (`false`) |

### `usb_serial_troubleshooter_driver.py`

Parameters:

| Parameter | Type | Default | |
|---|---|---|---|
| `~port` | string | `/dev/ttyACM0` | Serial port of the troubleshooter (115200 baud) |
| `~serial_timeout` | double | none (blocking) | Timeout [s] of serial read/write; the launch file sets `1` |

### `usb_ppps_hub_driver.py`

Parameters:

| Parameter | Type | Default | |
|---|---|---|---|
| `~hub_location` | string or list of strings | (required) | Location(s) of the hub, as `uhubctl -l` |
| `~hub_port` | int or list | (required) | Port(s) of the hub, as `uhubctl -p`; the same length as `~hub_location` |
| `~uhubctl_executable` | string | `uhubctl` | Command (and options) of `uhubctl`; `~` is expanded |

The node checks the location and the port with `uhubctl` at start up and shuts down when they are invalid.

### `auto_power_cycle.py`

(ROS 1 only) Subscribed topics:

- `~monitored_topic` (any type): when no message comes for
  `(1 / ~monitored_topic_expected_hz) * (1 + ~monitored_topic_allowed_delay_cycles)` seconds, it calls the power
  cycle service, then waits `~monitored_topic_respawn_delay` seconds before the next check.

Services called:

- `~power_cycle` (`std_srvs/Trigger`): the power cycle service of a driver (waited for at start up)

Parameters:

| Parameter | Type | Default | |
|---|---|---|---|
| `~monitored_topic_expected_hz` | double | `10` | Expected rate [Hz] of the monitored topic |
| `~monitored_topic_allowed_delay_cycles` | double | `2.0` | Allowed delay, in cycles of the expected rate |
| `~monitored_topic_respawn_delay` | double | `10.0` | Time [s] to wait after a power cycle |
| `~init_with_power_cycle` | bool | `true` | Power cycle once at start up |
