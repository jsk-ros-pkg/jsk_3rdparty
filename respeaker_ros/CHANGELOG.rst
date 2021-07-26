^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package respeaker_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------
* [respeaker_ros] Specify correct Python version in package.xml (`#247 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/247>`_)

  * Fixes typo introduced in https://github.com/jsk-ros-pkg/jsk_3rdparty/commit/39be21894a38112a1633cf8385caf079c35536ff

* Add respawn_delay to respeaker_node.py to reduce CPU load (`#241 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/241>`_)

   * Kill respeaker_node.py when USBError occur to reduce CPU load

* [respeaker_ros] fix default sound_play action name of speech_to_text.py (`#239 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/239>`_)
* Mistakes I found when I tried to use respeaker_ros on hirovision (`#217 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/217>`_)

  * add speech_recognition_msgs
  * change the name from respeaker.launch to sample_respeaker.launch

* Contributors: Koki Shinjo, Miyabi Tanemoto, Naoya Yamaguchi, Shun Hasegawa

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * respeaker_ros: use catkin_install_python to install scripts
  * fix 2to3, with print, raise, exception
  * use package.xml format 3 for package contains python depends

* [respeaker_ros] use rospy.logerr instead of print (`#206 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/206>`_)
* Add args to ros_speech_recognition (`#197 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/197>`_)

  * [respeaker_ros] update run_depend and test file to pass travis test properly

* Contributors: Kei Okada, Naoya Yamaguchi, Shingo Kitagawa

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------
* [respeaker_ros] increase timeout to pass the test https://api.travis-ci.org/v3/job/554008643/log.txt
* Contributors: Kei Okada

2.1.15 (2019-12-12)
-------------------
* [respeaker_ros] add pixel-ring in run_depend of respeaker_ros (`#184 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/184>`_)
* [respeaker_ros] install config dir in respeaker_ros (`#185 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/185>`_)
* Contributors: Shingo Kitagawa

2.1.14 (2019-11-21)
-------------------

2.1.13 (2019-07-10)
-------------------
* [respeaker_ros] increase timeout to pass the test (`#170 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/170>`_)

  * default tts_action_names should be soundplay
  * [respeaker_ros] increase timeout to pass the test https://api.travis-ci.org/v3/job/554008643/log.txt
  * [respeaker_ros] add python-speechrocognition-pip to package.depends, because scripts/speech_to_text.py depends on it

* [respeaker_ros] Add test file for speech_to_text (`#164 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/164>`_)

  * add test file for speech_to_text

* [respeaker_ros] add tts_action_names param: do not listen when the robot is speaking either japanese or english (`#168 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/168>`_)

  * add tts_actions param: do not listen when the robot is speaking either japanese or english

* Contributors: Naoya Yamaguchi, Shingo Kitagawa

2.1.12 (2019-05-25)
-------------------
* Make sample_respeaker.launch re-usable (`#161 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/161>`_)

  * [respeaker_ros] add docs for each args in sample_respeaker.launch
  * make sample_respeaker.launch re-usable

* respeaker_ros: cleanup error messages (`#155 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/155>`_)
* Contributors: Yuki Furuta, Kei Okada, Naoya Yamaguchi

2.1.11 (2018-08-29)
-------------------
* Add respeaker_ros package (`#152 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/152>`_)
* Contributors: Yuki Furuta

2.1.10 (2018-04-25)
-------------------

2.1.9 (2018-04-24)
------------------

2.1.8 (2018-04-17)
------------------

2.1.7 (2018-04-09)
------------------

2.1.6 (2017-11-21)
------------------

2.1.5 (2017-11-20)
------------------

2.1.4 (2017-07-16)
------------------

2.1.3 (2017-07-07)
------------------

2.1.2 (2017-07-06)
------------------

2.1.1 (2017-07-05)
------------------

2.1.0 (2017-07-02)
------------------

2.0.20 (2017-05-09)
-------------------

2.0.19 (2017-02-22)
-------------------

2.0.18 (2016-10-28)
-------------------

2.0.17 (2016-10-22)
-------------------

2.0.16 (2016-10-17)
-------------------

2.0.15 (2016-10-16)
-------------------

2.0.14 (2016-03-20)
-------------------

2.0.13 (2015-12-15)
-------------------

2.0.12 (2015-11-26)
-------------------

2.0.11 (2015-10-07 14:16)
-------------------------

2.0.10 (2015-10-07 12:47)
-------------------------

2.0.9 (2015-09-26)
------------------

2.0.8 (2015-09-15)
------------------

2.0.7 (2015-09-14)
------------------

2.0.6 (2015-09-08)
------------------

2.0.5 (2015-08-23)
------------------

2.0.4 (2015-08-18)
------------------

2.0.3 (2015-08-01)
------------------

2.0.2 (2015-06-29)
------------------

2.0.1 (2015-06-19 21:21)
------------------------

2.0.0 (2015-06-19 10:41)
------------------------

1.0.71 (2015-05-17)
-------------------

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05 12:28)
-------------------------

1.0.68 (2015-05-05 09:49)
-------------------------

1.0.67 (2015-05-03)
-------------------

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------

1.0.63 (2015-02-19)
-------------------

1.0.62 (2015-02-17)
-------------------

1.0.61 (2015-02-11)
-------------------

1.0.60 (2015-02-03 10:12)
-------------------------

1.0.59 (2015-02-03 04:05)
-------------------------

1.0.58 (2015-01-07)
-------------------

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20 16:01)
-------------------------

1.0.50 (2014-10-20 01:50)
-------------------------

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------

1.0.46 (2014-10-03)
-------------------

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26 09:17)
-------------------------

1.0.43 (2014-09-26 01:08)
-------------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------

1.0.37 (2014-09-08)
-------------------

1.0.36 (2014-09-01)
-------------------

1.0.35 (2014-08-16)
-------------------

1.0.34 (2014-08-14)
-------------------

1.0.33 (2014-07-28)
-------------------

1.0.32 (2014-07-26)
-------------------

1.0.31 (2014-07-23)
-------------------

1.0.30 (2014-07-15)
-------------------

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------

1.0.27 (2014-06-10)
-------------------

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------

1.0.22 (2014-05-22)
-------------------

1.0.21 (2014-05-20)
-------------------

1.0.20 (2014-05-09)
-------------------

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19 23:29)
-------------------------

1.0.15 (2014-04-19 20:19)
-------------------------

1.0.14 (2014-04-19 12:52)
-------------------------

1.0.13 (2014-04-19 11:06)
-------------------------

1.0.12 (2014-04-18 16:58)
-------------------------

1.0.11 (2014-04-18 08:18)
-------------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.7 (2014-04-10)
------------------

1.0.6 (2014-04-07)
------------------

1.0.5 (2014-03-31)
------------------

1.0.4 (2014-03-29)
------------------

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
