^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package downward
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------
* Fix for ROS-O (`#515 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/515>`_)

  * [.github/workflows/config.yml: disable installing recommends in apt-get install
  * [3rdparty/downward/package.xml: add linbfl-dev for FlexLexer.h

* Support ros-o / Ubuntu 22.04 (`#512 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/512>`_)

  * [downward] [ros-o] add -Wno-maybe-uninitialized
  * [downward] patching to fix std::vector namespace

* Contributors: Kei Okada, Yoshiki Obinata

2.1.28 (2023-07-24)
-------------------

2.1.27 (2023-06-24)
-------------------

2.1.26 (2023-06-14)
-------------------
* add LICENSE files (`#476 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/476>`_)
* Contributors: Kei Okada

2.1.25 (2023-06-08)
-------------------

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * downward: fix for python3, remove time.clock() and import commands
  * fix for python2/3 with ROS_PYTHON_VERSION
  * downward: -Wno-error=deprecated-copy is only for noetic
  * add -Wno-error=deprecated-copy for 20.04

* Contributors: Kei Okada

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

2.1.12 (2019-05-25)
-------------------

2.1.11 (2018-08-29)
-------------------
* downward: compile with -Wno-maybe-uninitialized to avoid error for 18.04 (`#154 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/154>`_)
* Contributors: Kei Okada

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
* remove both "g++" "g++-static" from package.xml (`#129 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/129>`_)
* Contributors: Kei Okada

2.1.5 (2017-11-20)
------------------
* change from g++-static to g++ (`#115 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/115>`_)
* Contributors: Kei Okada

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
* add gawk to run_depend (`#85 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/85>`_)
* Contributors: Kei Okada

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
* [CMakeLists.txt] downward: use http instead of https
* [CMakeLists.txt] Set timeout for downloading downward
* [package.xml] add ca-certificates to build_depends
* Contributors: Kei Okada, Kentaro Wada

2.0.13 (2015-12-15)
-------------------

2.0.12 (2015-11-26)
-------------------
* package.xml : downward uses time command on running (https://github.com/jsk-ros-pkg/jsk_planning/pull/30#issuecomment-146065373)
* Contributors: Kei Okada

2.0.11 (2015-10-07)
-------------------
* [travis.yml&downward] install time from rosdep instead of travis.yml
* Contributors: Yuki Furuta, Ryohei Ueda

2.0.10 (2015-10-07)
-------------------
* - [downward] use ExternalProject instead of mk to reduce build failure
  - [downward] add test for planning
  - [travis.yml] fix: add EXTRA_DEB=time for test downward
* Contributors: Yuki Furuta

2.0.9 (2015-09-26)
------------------
* [downward] Use rawgit instead of github to download downward
* Contributors: Ryohei Ueda

2.0.8 (2015-09-15)
------------------

2.0.7 (2015-09-14)
------------------

2.0.6 (2015-09-08)
------------------
* [downward/CMakeLists.txt] fix copy source directory path
  source path changed maybe from when we use jsk-ros-pkg/archives
* Contributors: Yuki Furuta

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
* [downward] use xzf to extract .tar.gz, instaed of xvzf to reduce log length
* Contributors: Kei Okada

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------
* Correct recursive call to make in downward
* Add build_depend on g++-static for downward
* Contributors: Scott K Logan

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
* [downward] Remove multilib from it's dependency
* use jsk-ros-pkg/archives for download tgz
* downward: Use native bitwidth when compiling
  Previous behavior is to always compile 32-bit binaries
* [downward] Ignore error when building downward.
* Contributors: Ryohei Ueda, Scott K Logan, Kei Okada

1.0.58 (2015-01-07)
-------------------

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------
* fix downward copy directory
* Contributors: Yuki Furuta

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
* disabel downward compiling
* Contributors: Ryohei Ueda

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------
* use tgz to download source
* Contributors: Kei Okada

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

1.0.41 (2014-09-23)
-------------------

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
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
* install downward
* Contributors: Kei Okada

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.5 (2014-03-31)
------------------
* Added missing find_package to downward
* Contributors: Scott K Logan

1.0.4 (2014-03-27)
------------------
* Added missing build_depends on rospack, roslib and mk
* Contributors: Scott K Logan
* downward: catkinize
* Contributors: Kei Okada

1.0.3 (2014-03-19)
------------------
* update revision number to 1.0.3

1.0.0 (2014-03-05)
------------------
* added dependency
* added downward
* Contributors: mikita
