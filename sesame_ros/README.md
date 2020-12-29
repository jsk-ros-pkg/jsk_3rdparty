# sesame_ros

ROS API for Sesame smart lock made by CANDY HOUSE, Inc.

![](images/sesame_ros_system.png)


#### API key

You have to create your authorization token by logging into the CANDY HOUSE Dashboard: https://my.candyhouse.co/

Note that you must be the owner of your Sesame in order to use Web API.

Please make sure you can see the list of Sesames on CANDY HOUSE Dashboard.


#### Python Package Requirements

This package supports [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv).

So pip dependencies written in `requirements.txt` will be automatically installed into virtualenv during build.


## Scripts in this package

- ### list_sesames.py

    A script for listing your Sesames.

    __Arguments__

    - `auth_token` (__required__): Your authorization token. Path to its file is also supported.


- ### sesame_server.py

    A node for controlling your Sesame.

    __Parameters__

    - `~auth_token` (`string`, __required__): Your authorization token. Path to its file is also supported.

    - `~device_id` (`string`, optional): UUID of your Sesame.

    - `~nickname` (`string`, optional): Nickname of your Sesame.

    `~device_id` and `~nickname` are used for searching Sesame from your Sesame list.

    If neither of them matches or is specified, the first found Sesame will be used.

    - `~command_timeout` (`int` or `float`, optional): Timeout for executing commands. Default is 60 [sec].


    __Advertising services (See Usage section)__

    - `~get_status` (`sesame_ros/Status`)

    - `~lock` (`sesame_ros/Command`)

    - `~unlock` (`sesame_ros/Command`)

    - `~force_sync` (`sesame_ros/Command`)


## Defined service list

- __sesame_ros/Status__

```
---
string nickname
string serial
string device_id
int32 battery
bool locked
bool responsive
```

- __sesame_ros/Command__

```
---
string status
bool successful
string error
```


## Usage

### To list your Sesames:

Just run below.
```
$ rosrun sesame_ros list_sesames.py YOUR_AUTH_TOKEN
```

### To controll your Sesame:

1. Start service server.
    ```
    $ roscore
    $ rosrun sesame_ros sesame_server.py _auth_token:=YOUR_AUTH_TOKEN _device_id:=YOUR_SESAME's_UUID
    ```

2. Call service.

    - To get your Sesame status:
        ```
        $ rosservice call /sesame_server/get_status
        ```

    - To lock your Sesame:
        ```
        $ rosservice call /sesame_server/lock
        ```

    - To unlock your Sesame:
        ```
        $ rosservice call /sesame_server/unlock
        ```

    - To force the server to update your Sesame status:
        ```
        $ rosservice call /sesame_server/force_sync
        ```
