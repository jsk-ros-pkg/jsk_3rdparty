^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package julius_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* [ros-o] julius: use system install julius (`#518 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/518>`_)
* julius_ros/scripts/run_julius.sh: more information on how to install julius and grammer/dictation kit
* [ros-o] julius: use system install julius, download dictation and grammer kit by script
* Contributors: Kei Okada

2.1.29 (2025-01-05)
-------------------
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
* Pr/use sound themes freedesktop (`#472 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/472>`_)
* add test to check if ros node is loadable (`#463 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/463>`_)
* Contributors: Kei Okada, Koki Shinjo

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------
* Add DNN version julius (`#259 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/259>`_)

  * Fix order of args in julius.launch
  * Update README
  * Do not use git-lfs and revert unnecessary change
  * Add julius_ros test for DNN version
  * Use audio port instead of microphone input
  * Update README and julius.launch arg doc
  * Use Julius config file in julius_ros
  * Fix typo: input audio via port

* Contributors: Naoya Yamaguchi

2.1.22 (2021-06-10)
-------------------

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------
* fix typo in julius_client.py (`#203 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/203>`_)
* add more arg options for julius.launch (`#144 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/144>`_)

  * add julius_output args
  * add julius_args
  * add args for device,channels,depth,sample_rate,format

* Contributors: Kei Okada, Shingo Kitagawa

2.1.18 (2020-07-20)
-------------------
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * julius_ros: fix for python3, specially str <-> bytes
  * fix 2to3, with print, raise, exception
  * use package.xml format 3 for package contains python depends

* Contributors: Kei Okada

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------

2.1.15 (2019-12-12)
-------------------

2.1.14 (2019-11-21)
-------------------
* set SoundRequest.volume for kinetic (`#173 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/173>`_)
* Contributors: Kei OKada

2.1.13 (2019-07-10)
-------------------

2.1.12 (2019-05-25)
-------------------

2.1.11 (2018-08-29)
-------------------

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
* [julius_ros] set timeout to self.play_sound(self.start_signal) (`#116 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/116>`_)
* Contributors: Kanae Kochigami

2.1.3 (2017-07-07)
------------------

2.1.2 (2017-07-06)
------------------

2.1.1 (2017-07-05)
------------------
* [julius_ros] fix: missing deps julius-voxforge (`#109 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/109>`_)
* Contributors: Furushchev

2.1.0 (2017-07-02)
------------------
* [julius_ros] support grammatical recognition (`#102 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/102>`_)
  * [julius_ros] fix: initial vocabulary
  * [julius_ros][julius_client.py] advertise service on grammar mode
  * [julius_ros][julius.test] delay play audio 10 seconds
  * [julius_ros] add missing deps
  * [julius_ros] split grammar test
  * [julius_ros] support grammar
  * [julius_ros] update conf for grammar recognition
    [julius_ros] escape xml value before parse
    [julius_ros] update launch files
    [julius_ros] use machine tag by default
    [julius_ros] support respawn; minor fix
    [julius_ros][julius_grammar.launch] add argument for topic name of 'speech_to_text'
    [julius_ros] add command line tools to add grammar / vocabulary to julius engine
    [julius_ros][julius_client.py] add service to show julius engine status
    [julius_ros][julius_client.py] bugfix: INPUTONCHANGE WAIT
    [julius_ros][julius_client.py] cleanup change gram

* [julius_ros] Update julius to 4.4.2 / add ROS interface (`#99 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/99>`_)
  * add julius_ros package 
  * [julius_ros] add test

* Contributors: Furushchev, Yuki Furuta

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
