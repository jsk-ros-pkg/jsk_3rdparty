^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package collada_urdf_jsk_patch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* [ros-o] collada_urdf_jsk_patch: skip cmake/pkg config generation (`#517 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/517>`_)
* Contributors: Kei Okada

2.1.29 (2025-01-05)
-------------------
* Support ros-o / Ubuntu 22.04 (`#512 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/512>`_)

  * [collada_urdf_jsk_patch] [ros-o] add CATKIN_IGNORE only when ROS-O, use STREQUAL to compare ROS_DISTRO in cmake
  * [collada_urdf_jsk_patch] [ros-o] ignore it because no longer needed

* Contributors: Yoshiki Obinata

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

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------

2.1.21 (2020-08-19)
-------------------
* add missing packages, closes https://github.com/ros/rosdistro/pull/26216 (`#211 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/211>`_)

* Contributors: Kei Okada

2.1.20 (2020-08-07)
-------------------
* apply fix_issue_18 only for collada_urdf 1.12.12 (`#209 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/209>`_)

  * add https://github.com/Naoki-Hiraoka/collada_urdf/commit/c37592e86af2d949479e3db9e271e34ff8eff189
  * use collada_urdf 1.12.12 for melodic and later
  * fix bug introduced in 0c200c7ce26cdf3c16f36cf5dd68d05ee06775e2

* Contributors: Kei Okada

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * collada_urdf_jsk_patch : use download_unpack_build instead of git_checkout
  * remove collada_urdf from find_packaeg(catkin) unitl assimp_devel v5.0.1 released
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

2.1.11 (2018-08-29)
-------------------
* collada_urdf_jsk_patch: std=gnu++11 need for kinetic and later (`#154 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/154>`_)
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
* [collada_urdf_jsk_patch] fix: occasional build failure (`#105 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/105>`_)
  * [collada_urdf_jsk_patch] fix: no notify catkin build in catkin build
  * [collada_urdf_jsk_patch] fix occasional build failure
* Contributors: Yuki Furuta

2.0.20 (2017-05-09)
-------------------
* use indigo-devel before changing hydro-devel package structure
* Contributors: Kei Okada

2.0.19 (2017-02-22)
-------------------
* update patch for https://github.com/ros/robot_model/commit/3e5a220 (`#86 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/86>`_)
  * [collada_urdf_jsk_patch][README.md] update readme
  * [collada_urdf_jsk_patch][use_assimp_devel.patch] update patch according to latest commit at collada_urdf
  * [collada_urdf_jsk_patch][Makefile] don't apply collada_urdf_latest_gazebo.patch (see https://github.com/ros/robot_model/commit/3e5a220a67cf063d1e389cfbce3f05147c46f547)
* Contributors: Yuki Furuta

2.0.18 (2016-10-28)
-------------------
* fix for kinetic (`#78 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/78>`_)
  * collada_urdf_jsk_patch/CMakeLists.txt: from kinetc we need to use c++11
  * collada_urdf_jsk_patch, fix condition >= indigo
* Contributors: Kei Okada

2.0.17 (2016-10-22)
-------------------

2.0.16 (2016-10-17)
-------------------

2.0.15 (2016-10-16)
-------------------
* collada_urdf_jsk_patch : compile with -v -i
* Contributors: Kei Okada

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
* [collada_urdf_jsk_patch] Add README.md
* Contributors: Ryohei Ueda

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
* [collada_urdf_jsk_patch] Depends on catkin-tools
* Contributors: Ryohei Ueda

2.0.4 (2015-08-18)
------------------
* collada_urdf_jsk_patch/Makefile: use catkin build to compile robot_model
* Contributors: Kei Okada

2.0.3 (2015-08-01)
------------------

2.0.2 (2015-06-29)
------------------
* [/jsk_ros_patch/collada_urdf_jsk_patch/CMakeLists.txt] if ROS_DISTRO is jade, the we'll use robot_model for indigo. jade-devel branch for robot_model is not released yet
* Contributors: Kei Okada

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
* [collada_urdf_jsk_patch] Remove urdf and urdfdom from package.xml
  because it should be resolved via collada_urdf package
* Contributors: Ryohei Ueda

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
* add patch for removing old gazebo settings
* Contributors: Yohei Kakiuchi

1.0.55 (2014-12-09)
-------------------
* fix compiling jsk_ros_patch for indigo
* Contributors: Yohei Kakiuchi

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
* Fix a logic error in collada_jsk_patch
  This fixes a regression caused by 9846892b8ec1c1b3e655015298cd9a8e17b155e7
* Contributors: Scott K Logan

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------
* disable collada_urdf_jsk_patch for indigo
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
* install collada_to_urdf

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
* copy collada_to_urdf binary to devel directory
* Contributors: Masaki Murooka

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------
* disable ssl setting for download robot_model
* Contributors: Kei Okada

1.0.16 (2014-04-19)
-------------------
* add depends to collada_parser, collada_urdf, urdf and kdl_parser
* Contributors: Kei Okada

1.0.15 (2014-04-19)
-------------------
* add depend to class_loader, pluginlib, rostest
* Contributors: Kei Okada

1.0.14 (2014-04-19)
-------------------
* add missing deps(mk,git,..) to collada_urdf_jsk_patch
* Contributors: Kei Okada

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------
* update collada_urdf to use assimp_devel on hydro-devel
* Contributors: Kei Okada

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.6 (2014-04-07)
------------------
* fix to work with hydro (which uses same setup with groovy)
* Contributors: Kei Okada

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* use rosdep instead of depend package
* set target name as urdf_to_collada
* copy urdf_to_collada bin file to CATKIN_PACKAGE_BIN_DESTINATION
* add caktin buildtool_depend and find_package, catkin_package
* catkinize collada_urdf_jsk_patch
* change robot_model repository from kforge to github on fuerte, [`#227 <https://github.com/jsk-ros-pkg/jsk_common/issues/227>`_]
* pull request merged ( https://github.com/ros/robot_model/commit/2eaf5c9166ebd50cbc14cf807d3d09b0597ee045 )
* add collada_cmake.patch for compiling on groovy
* add set_url_name_groovy.patch for compiling on groovy
* revert set_url_name.patch for compiling on fuerte
* add temporary patch for using multiple visual, it pull requested at https://github.com/ros/robot_model/pull/20
* update for using repository in github
* fix for assimp3 which aiScene is hiden
* fix for groovy
* download collada-dom-2.2.zip from jsk-ros-pkg, pr2.willowgarage.com has stopped?
* robot_model repository moved to github, temporary using latest hg repository
* use collada-dom 2.4 for groovy
* fix HG_ROS_PACKAGE_PATH -> ROS_PACKAGE_PATH
* fix for groovy
* use PLATFORM_FLOAT64 for daeFloat, collada-fom for groovy uses -DCOLLADA_DOM_DAEFLOAT_IS64, update pr2.l to use double precision value
* fix segfault on groovy problem https://github.com/ros/robot_model/issues/4
* fix to compile on groovy?
* add ColladaDOM150 namespace
* fix for groovy
* fix to compile on groovy
* fix to compile on groovy
* fix to compile on groovy
* use http instead of https to avoid certificate verify failure
* add set_url_name patch
* clean up and force remove urdf_to_collada when make clean
* fix Makefile error in collada_urdf_jsk_patch
* fix for hg https://code.ros.org/trac/ros/ticket/3748
* use robot_model version from rosversion
* update to electric
* fix download robot_model-1.5.1_hg
* rename colada_urdf_hg to robot_model-1.5.1_hg
* fix Makefile syntax error
* fix to work with electric : hg_checkout.mk is changed
* make clean to remove rosdep.yaml files
* set HG_REVISION not HG_BRANGE
* update to hg repository
* update tags cturtle->robot_model-1.4.0
* add debian info to rosdep.yaml
* collada format uses degree for upper and lower limits
* add radlimit patch to output limit in radius
* get geometry data from geometry.get instead of urdf_link->visual for SPHERE,BOX,CYLINDER
* add more error checking to avoid segfault
* update not to run rosmake in Makefile
* collada_urdf_jsk_patch does not depends on collada_urdf
* add jsk patch for collada_urdf, that support material, cube, cylinder, sphere
* Contributors: Ryohei Ueda, Kei Okada, youhei
