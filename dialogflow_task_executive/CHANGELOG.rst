^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dialogflow_task_executive
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* Add ROS-O 24.04 test (`#521 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/521>`_)
* Contributors: Kei Okada

2.1.29 (2025-01-05)
-------------------
* Support ros-o / Ubuntu 22.04 (`#512 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/512>`_)

  * [respeaker_ros][diamondback] [ros-o] only use STREQUAL to compare ROS_DISTRO in cmake

* fix CMake: install apps/samples/config (`#504 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/504>`_)
* GA: relax failed test dialogflow_task_executive, julius_ros (`#497 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/497>`_)
* Contributors: Kei Okada

2.1.28 (2023-07-24)
-------------------

2.1.27 (2023-06-24)
-------------------
* fix package.xml/CMakeLists.txt to supress catkin_lint errors (`#479 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/479>`_)
* Contributors: Kei Okada

2.1.26 (2023-06-14)
-------------------
* add LICENSE files (`#476 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/476>`_)
* Contributors: Kei Okada

2.1.25 (2023-06-08)
-------------------
* Add option to use project_id from json, instead of credentials (`#460 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/460>`_)
* add test to check if ros node is loadable (`#463 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/463>`_)

  * install python files under CATKIN_PACKAGE_BIN_DESTINATION
  * dialogflow_task_executive: add missing dependencies to package.xml
  * dialogflow_task_executive: pip -i only works with 64 bit (x86_64 and aarch64), other archtecture skips rospy_node.test
  * dialogflow_task_executive/CMakeLists.txt: test/test_rospy_node.test depends on generate_messages
  * dialogflow_task_executive/requirements.txt.indigo: specify grpcio for indigo
  * add catkin_install_python for test, it is also change installed directory from BIN to SHARE, because we want to have same directory structure between devel and install
  * add test to check if ros node is loadable
    If we use python2 PYTHON_INTERPRETER on 20.04, python2 fails to load rospy in /opt/ros/noetic, because rospy moduels are alraedy updated.
    If we use python3 PYTHON_INTERPRETER on 18.04, python3 can load rospy in /opt/ros/melodic by chance.
    c.f. https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/367

* [dialogflow_task_executive] chmod -x because of catkin_virtualenv (`#449 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/449>`_)

* [dialogflow_task_executive] not use map for python3 in task_executive (`#447 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/447>`_)
* [dialogflow_task_executive] fix typo in dialogflow_client.py (`#445 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/445>`_)

  * chmod -x because of catkin_virtualenv
  * not use map for python3 in task_executive
  * fix typo in dialogflow_client.py

* [dialogflow_task_executive] add sample apps (`#293 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/293>`_)
* [dialogflow_task_executive] add speech_to_text_other topic name in speech_to_text_mux (`#302 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/302>`_)
* [CI] build catkin_virtualenv packages in indigo. (`#419 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/419>`_)
* [dialogflow_task_executive] check if ROS_PYTHON_VERSION is set to support indigo in dialogflow_client (`#408 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/408>`_)
* [`#405 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/405>`_ and `#410 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/410>`_] Fix CI (`#415 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/415>`_)
* [dialogflow_task_executive] Support noetic (`#362 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/362>`_)
* Fix GithubAction (`#386 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/386>`_)
* [dialogflow_task_executive] Enable aarch64 (`#364 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/364>`_)
* GithubAction: add test for  aarch64(melodic) / indigo (arm64) (`#365 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/365>`_)
* [dialogflow_task_executive] set optenv as default false (`#368 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/368>`_)
* Explicit python interpreter in catkin_virtualenv (`#367 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/367>`_)
* [dialogflow_task_executive] add comments in DialogResponse.msg (`#366 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/366>`_)
* [dialogflow_task_executive] adding some args in dialogflow_ros.launch (`#361 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/361>`_)
* [dialogflow_task_executive] fix warn bug when got unknown action (`#363 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/363>`_)
* fix type on dialogflow_task_executive/requirements.txt.indigo (`#355 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/355>`_)
* [dialogflow_task_executive] add requirements.txt in .gitignore (`#353 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/353>`_)
* [dialogflow_task_executive] add doc in launch (`#349 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/349>`_)
* [dialogflow_task_executive]separate dialogflow API client and app execution from dialogflow_task_executive (`#343 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/343>`_)
* dialogflow_task_executive: re-enable idnigo (`#342 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/342>`_)
* dialogflow_task_executive: more depends and udpate README.md (`#334 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/334>`_)
* [dialogflow_task_executive] Fix bugs when subscribing /text (std_msgs/String) topic (`#328 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/328>`_)
* [dialogflow_task_executive] load dialogflow project_id from credentials (`#319 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/319>`_)
* [dialogflow_task_executive] Fix merging `#317 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/317>`_ (`#318 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/318>`_)
* Support dialogflow hotword yaml (`#307 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/307>`_)
* add sample code of dialogflow (`#317 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/317>`_)

* Contributors: Kei Okada, Koki Shinjo, Naoto Tsukamoto, Naoya Yamaguchi, Shingo Kitagawa, Yoshiki Obinata, Iory Yanokura

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------
* [dialogflow_task_executive] Change mux namespace for speech_to_textf( `#221 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/221>`_)
* Contributors: Naoya Yamaguchi

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------
* [dialogflow_task_executive] accepct full name of app name for dialogflow action (`#208 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/208>`_)

  * update readme to accept dialogflow action name
  * use whole app_name for action

* Contributors: Kei Okada, Shingo Kitagawa

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------

2.1.15 (2019-12-12)
-------------------
* add url in dialogflow_task_executive (`#181 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/181>`_)
* Contributors: Shingo Kitagawa

2.1.14 (2019-11-21)
-------------------
* [dialogflow_task_executive] add dialogflow_task_executive (`#165 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/165>`_)
  * add std_msgs in package.xml (`#177 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/177>`_)
  * update system fig/img
  * add jsk-dialog.conf example
  * add upstart example

* Contributors: Shingo Kitagawa
