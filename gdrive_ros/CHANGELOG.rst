^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gdrive_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------

2.1.28 (2023-07-24)
-------------------
* use query with folder + title, instead of list all files then apply filters (`#481 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/481>`_)
* Contributors: Kei Okada

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
* gdrive_ros: use virtualenv (`#458 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/458>`_)

  * use python3 for pydrive_ros
    latest pydrive requires newer rsa??
  * gdrive_ros: catkin_virtualenv changes mode to 100644
  * gdrive_ros: enable to use relative path for settings_yaml
  * gdrive_ros: use catkin_virtualenv

* [gdrive_ros] fix sys.version error (`#429 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/429>`_)
* [gdrive_ros] Add ros client library and examples ( for roseus and rospy) (`#295 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/295>`_)

  * euslisp directory is installed via 'install(DIRECTORY euslisp', so we can remove 'install(PROGRAMS euslisp/sample-gdrive-ros-client.l'
  * [gdrive_ros] fix module import
  * [gdrive_ros] add rospy client and update examples
  * [gdrive_ros] update CMakeLists.txt to add installation of euslisp and sample
  * [gdrive_ros] Add a roseus client library and sample
  * [gdrive_ros] add gdrive_ros client

* [gdrive_ros] add machine arg in gdrive_server.launch (`#288 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/288>`_)

  * add machine arg in gdrive_server.launch

* Contributors: Aoi Nakane, Kei Okada, Koki Shinjo, Naoto Tsukamoto, Shingo Kitagawa

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------
* add rostest in test_depend (`#263 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/263>`_)
* fix-typo-gdrive (`#272 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/272>`_)
* add version check at top of gdrive_server_node (`#270 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/270>`_)

* Contributors: Koki Shinjo, Shingo Kitagawa

2.1.22 (2021-06-10)
-------------------
* set max authentication trial times as infinite (`#249 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/249>`_)
* catch ServerNotFoundError in gdrive_server_node.py ((`#240 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/240>`_)
* update doc links in gdrive_ros (`#233 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/233>`_)
* [gdrive_ros] add node_name arg in gdrive_server.launch (`#234 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/234>`_)
* add roslaunch test (`#232 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/232>`_)
* [gdrive_ros] add argument respawn (`#230 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/230>`_)

* Contributors: Shingo Kitagawa

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------
* [gdrive_ros] Fix gdrive_ros to catch error properly (`#205 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/205>`_)

  * fix ros log format in gdrive_ros
  * add oserr when file not found in gdrive_ros

* Contributors: Kei Okada, Shingo Kitagawa

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------

2.1.15 (2019-12-12)
-------------------
* add gdrive_ros package (`#182 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/182>`_)
* Contributors: Shingo Kitagawa
