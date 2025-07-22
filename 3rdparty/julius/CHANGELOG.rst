^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package julius
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* [ros-o] julius: use system install julius, download dictation and grammer kit by script (`#518 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/518>`_)
* Contributors: Kei Okada

2.1.29 (2025-01-05)
-------------------

2.1.28 (2023-07-24)
-------------------
* fix build farm (`#487 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/487>`_)
  * julius: fix location URLs, because osdn is too slow

* Contributors: Kei Okada

2.1.27 (2023-06-24)
-------------------

2.1.26 (2023-06-14)
-------------------
* add LICENSE files (`#476 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/476>`_)
* Contributors: Kei Okada

2.1.25 (2023-06-08)
-------------------
* julius: Use DEB_TARGET_GNU_TYPE to fix https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/467 (`#470 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/470>`_)
* GA: enable melodic/aarch64 (`#432 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/432>`_)
* fix catkin build stacks in GA (`#316 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/316>`_)

  * use wget to download from osdn, to use mirror site
  * download .zip file from https://osdn.net/dl/julius/

* Contributors: Aoi Nakane, Kei Okada, Naoto Tsukamoto, Naoya Yamaguchi, Shingo Kitagawa

2.1.24 (2021-07-26)
-------------------
* use latest config.guess.patch for https://github.com/ros/rosdistro/pull/30279 (`#275 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/275>`_)

* Contributors: Kei Okada

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

2.1.10 (2018-04-25)
-------------------

2.1.9 (2018-04-24)
------------------

2.1.8 (2018-04-17)
------------------
* julius: add wget to dependencies (`#138 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/138>`_)
* Contributors: Yuki Furuta

2.1.7 (2018-04-09)
------------------
* julius: add rsync & unzip to run_depend (`#134 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/134>`_)
* Contributors: Yuki Furuta

2.1.6 (2017-11-21)
------------------

2.1.5 (2017-11-20)
------------------

2.1.4 (2017-07-16)
------------------

2.1.3 (2017-07-07)
------------------
* add unzip to build_depend ( `#114 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/114>`_ )
* Contributors: Kei Okada

2.1.2 (2017-07-06)
------------------
* [julius][package.xml] add rsync to run_depend (`#112 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/112>`_)
* Contributors: Yui Furuta

2.1.1 (2017-07-05)
------------------
* [julius] fix: failure on buildfirm (`#109 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/109>`_)
* Contributors: Yuki Furuta

2.1.0 (2017-07-02)
------------------
* [julius] update to use julius v4.4.2 (`#99 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/99>`_)
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
* catkinize julius
* Contributors: Yuki Furuta
