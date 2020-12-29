^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libsiftfast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * fix for python2/3 with ROS_PYTHON_VERSION
  * fix libsiftfast for python3 / boost 1.71
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

2.1.13 (2019-07-10)
-------------------

2.1.12 (2019-05-25)
-------------------
* enable to compile libsiftfast with current numpy.get_include() (`#162 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/162>`_ )
* Contributors: Kei Okada

2.1.11 (2018-08-29)
-------------------
* fix for melodic (`#154 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/154>`_)
  * libsiftfast : add 02.cmake_warn_narrowing.patch 03.skip_python_bindings.patch.bak 04.boost_65_numpy_1_10.patch for 18.04
  * libsiftfast: patch all fiels within pathes directory
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
* [libsiftfast] find python 2 (`#106 <https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/106>`_)
  * libsiftfast is written for python2 and can be successfully installed with python2.
  * However this package finds and uses python3 (not 2) first on environment where both python2 and 3 are installed. (In new travis environment both python2 and 3 seems to be installed)
  * This issue is fixed in this PR by setting version 2 on find_package python.
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
* Use PYTHON_INSTALL_DIR to install python module with catkin
  Modified:
  - 3rdparty/libsiftfast/CMakeLists.txt
* Contributors: Kentaro Wada

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
* [libsiftfast] Install python binding correctly when catkin config --no-install
* Contributors: Kentaro Wada

2.0.8 (2015-09-15)
------------------

2.0.7 (2015-09-14)
------------------
* [libsiftfast] Fix CMakeLists to generate python binding
* Contributors: Kentaro Wada

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
* change libsiftfast to non-catkin package by add SKIP_CMAKE_CONFIG_GENERATION
* Contributors: Kei Okada

1.0.8 (2014-04-11)
------------------
* Merge pull request #376 from k-okada/catkinize_lib_siftfast
* fix for buildpakcage: use install(CODE for libraries, since library file is generated during compile phase; remove devel directory when dhbuild; install share/siftfast -> share/libsiftfast
* Contributors: Kei Okada
* Only run Makefile during build phase (not install)
  Currently, `Makefile` is re-run when catkin installs the package. This causes `Makefile` to re-install, this time leaving the files in `/` instead of an intermediate directory. This ensures that once built, `Makefile` is not re-run.
* Contributors: Scott K Logan

1.0.7 (2014-04-10)
------------------
* Added missing build_depend on rospack and roslib
* Handle case where ROS_DISTRO is not set
* Contributors: Scott K Logan

1.0.6 (2014-04-07)
------------------
* catkinize libsiftfast, add fake add_library, set_target_properties for catkin, groovy does not suport EXPORTED_TARGETS
* Contributors: Kei Okada

1.0.0 (2014-03-05)
------------------
* add clean patched
* change SVN repository to new sourceforge server. Fixed https://code.google.com/p/rtm-ros-robotics/issues/detail?id=84
* moved posedetection_msgs, sift processing, and other packages to jsk_common and jsk_perception
* Contributors: Yusuke Furuta, rosen, Kei Okada
