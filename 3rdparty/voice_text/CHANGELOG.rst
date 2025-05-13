^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package voice_text
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------
* add LICENSE files
* Contributors: Kei Okada

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------

2.1.28 (2023-07-24)
-------------------

2.1.27 (2023-06-24)
-------------------

2.1.26 (2023-06-14)
-------------------

2.1.25 (2023-06-08)
-------------------
* GA: enable melodic/aarch64 (`#432 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/432>`_)
* Restart voice text if verification fails (`#300 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/300>`_)
* Add verification check rosinfo (`#298 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/298>`_)
* Contributors: Aoi Nakane, Kei Okada, Naoto Tsukamoto, Naoya Yamaguchi, Shingo Kitagawa, Yoshiki Obinata

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------
* [voice_text] Fix README to follow supporting other speakers than SAYAKA (`#220 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/220>`_)

* Contributors: Shun Hasegawa

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

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

2.1.14 (2019-11-21)
-------------------

2.1.13 (2019-07-10)
-------------------
* [voice_text] Call rosservice from python instead of bash (`#166 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/166>`_ )

  * rewrite text2wave with python
  * call rosservice from python instead of bash

* Contributors: Hideaki Ito

2.1.12 (2019-05-25)
-------------------
* Fix install directory of text2wave to ./lib -> ./bin (`#160 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/160>`_)
  `text2wave` Was wrongly  installed to `CATKIN_PACKAGE_LIB_DESTINATION`
  The launch file is assumed that it is installed under `rospack find voice_text`/bin
  https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/2.1.10/3rdparty/voice_text/launch/voice_text.launch#L29
* Contributors: Kei Okada

2.1.11 (2018-08-29)
-------------------

2.1.10 (2018-04-25)
-------------------
* add dependencies from voice_text to vt_dummy, solves `#139 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/139>`_ (`#143 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/143>`_)
* Contributors: Kei Okada

2.1.9 (2018-04-24)
------------------
* voice_text: add gencfg target to deps (`#141 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/141>`_)
* Contributors: Yuki Furta

2.1.8 (2018-04-17)
------------------

2.1.7 (2018-04-09)
------------------

* voice_text: support dynamic linking (`#135 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/135>`_)
  * install voice_text TARGETS - voice_text: CMakeLists.txt: remove debug code to force non-exists VT_LIB_PATH
  * voice_text: guide to install libs
  * install voice_text TARGETS
  * voice_text: CMakeLists.txt: remove debug code to force non-exists VT_LIB_PATH
  * add dependencies from generate_message_cpp to voice_text
  * use vt_dummy when we do not have voice_text library
* Contributors: Kei Okada, Yuki Furuta

2.1.6 (2017-11-21)
------------------

2.1.5 (2017-11-20)
------------------
* [voice_text] Add respawn argument for sound_play (`#125 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/125>`_)
* Contributors: Shunichi Nozawa

2.1.4 (2017-07-16)
------------------

2.1.3 (2017-07-07)
------------------

2.1.2 (2017-07-06)
------------------

2.1.1 (2017-07-05)
------------------
* add dynamic_reconfigure to run/build depends  (`#110 <https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/110>`_)
* Contributors: Kei Okada

2.1.0 (2017-07-02)
------------------
* [voice_text] Refactor API (`#101 <https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/101>`_)
  * Cleanup directory (remove rosbuild related files.)
  * Rewrote VoiceText server node as ROS friendly.
  * text2wave calls rosservice internally (This enables running VoiceText
  * engine on remote machine easily)
  * Create sample launch file
  * Create README
  * WARNING : This breaks API (we need to run voice_text node in addition to sound_play), so existing users will have to change launch file for using voice text.

* Contributors: Yuki Furuta

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
* voice_text : clean up CMakeList.txt
* [3rdparty/voice_text/text2wave] Enable text2wave using VoiceText other than pr2 robots.
* Contributors: Kei Okada, Shunichi Nozawa

2.0.14 (2016-03-20)
-------------------

2.0.13 (2015-12-15)
-------------------

2.0.12 (2015-11-26)
-------------------

2.0.11 (2015-10-07)
-------------------

2.0.10 (2015-10-07)
-------------------

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

2.0.1 (2015-06-19)
------------------

2.0.0 (2015-06-19)
------------------
* move from jsk_common to jsk_3rdparty

1.0.72 (2015-06-07)
-------------------

1.0.71 (2015-05-17)
-------------------

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05)
-------------------

1.0.68 (2015-05-05)
-------------------

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

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

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

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------

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

1.0.44 (2014-09-26)
-------------------

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------
* Support cakint for vice_text by using catkin_find command
* Contributors: Ryohei Ueda

1.0.41 (2014-09-23)
-------------------

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------
* catkinize python_twoauth and voice_text, modify multi_map_server's catkin.cmake
* Contributors: Ryohei Ueda, JSK applications

* catkinize python_twoauth and voice_text, modify multi_map_server's catkin.cmake
* Contributors: Yuto Inagaki

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
* add nkf to rosdep.yaml
* do not compile if voicetext is not installed
* add r58200(fixed include path), r58221(added nkf to rosdep) by mikita
* add voice_text client program, copy from jsk-ros-pkg-unreleased
* Contributors: Kei Okada
