^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sesame_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* CI: add ROS-O testing on arm (`#528 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/528>`_), fix sesame_ros on arm64
  * CI: add ROS-O testing on arm
  * CI: use ros-one-catkin-virtualenv
  * sesami_ros: add requirements.in.python3.12, for arm64 22.04/24.04
* Contributors: Kei Okada, Yoshiki Obinata

2.1.29 (2025-01-05)
-------------------
* Support ros-o / Ubuntu 22.04 (`#512 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/512>`_)

  * [sesame_ros] relax venv check
  * [sesame_ros] remove cffi which cannot be installed in Ubuntu 22.04 env & not used in sesame_ros

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
* add test to check if ros node is loadable (`#463 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/463>`_)
* Explicit python interpreter in catkin_virtualenv (`#367 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/367>`_)
* Contributors: Kei Okada, Shingo Kitagawa, Yoshiki Obinata

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

  * update sesame_ros/requirements.txt to pass https://github.com/locusrobotics/catkin_virtualenv/blob/master/README.md#locking-dependencies

* Contributors: Kei Okada

2.1.17 (2020-04-16)
-------------------
* update pip modules for security reason (`#196 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/196>`_)
* add idna==2.7
  to cloes ERROR: requests 2.20.0 has requirement idna<2.8,>=2.5, but you'll have idna 2.9 which is incompatible.
* Bump urllib3 from 1.22 to 1.24.2 in /sesame_ros
  Bumps [urllib3](https://github.com/urllib3/urllib3) from 1.22 to 1.24.2.
  - [Release notes](https://github.com/urllib3/urllib3/releases)
  - [Changelog](https://github.com/urllib3/urllib3/blob/master/CHANGES.rst)
  - [Commits](https://github.com/urllib3/urllib3/compare/1.22...1.24.2)
  Signed-off-by: dependabot[bot] <support@github.com>
* Bump requests from 2.19.1 to 2.20.0 in /sesame_ros
  Bumps [requests](https://github.com/psf/requests) from 2.19.1 to 2.20.0.
  - [Release notes](https://github.com/psf/requests/releases)
  - [Changelog](https://github.com/psf/requests/blob/master/HISTORY.md)
  - [Commits](https://github.com/psf/requests/compare/v2.19.1...v2.20.0)
  Signed-off-by: dependabot[bot] <support@github.com>
* Bump pyopenssl from 16.2.0 to 17.5.0 in /sesame_ros
  Bumps [pyopenssl](https://github.com/pyca/pyopenssl) from 16.2.0 to 17.5.0.
  - [Release notes](https://github.com/pyca/pyopenssl/releases)
  - [Changelog](https://github.com/pyca/pyopenssl/blob/master/CHANGELOG.rst)
  - [Commits](https://github.com/pyca/pyopenssl/compare/16.2.0...17.5.0)
  Signed-off-by: dependabot[bot] <support@github.com>
* Contributors: Kei Okada, dependabot[bot]

2.1.16 (2020-04-16)
-------------------
* add i386 tests (`#191 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/191>`_)
* specify python module number, closes `#190 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/190>`_
* Contributors: Kei Okada

2.1.15 (2019-12-12)
-------------------
* [sesame_ros] Add dependency for building cryptograpy (`#180 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/180>`_)

  * Add url of ROS wiki for sesame_ros
  * Add k-okada to maintainer of sesame_ros
  * Add libffi-dev and libssl-dev as dependencies of cryptography

* Contributors: Yuto Uchimi

2.1.14 (2019-11-21)
-------------------
* Add sesame_ros package (`#176 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/176>`_)
* Contributors: Yuto Uchimi
