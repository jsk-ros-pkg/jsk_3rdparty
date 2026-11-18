#ifndef ZDEPTH_IMAGE_TRANSPORT_ROS_VERSION_H_
#define ZDEPTH_IMAGE_TRANSPORT_ROS_VERSION_H_

// ROS1/ROS2 detection, as jsk_ros1_ros2_compat/ros_version.h (without
// depending on it): <ros/ros.h> of ROS1 (roscpp) defines ROS_VERSION_MAJOR
// to 1; on ROS2 it is not available, and the undefined ROS_VERSION_MAJOR is
// 0 in #if.
#if defined(__has_include)
#if __has_include(<ros/ros.h>)
#include <ros/ros.h>
#endif
#else
#include <ros/ros.h>
#endif

#endif  // ZDEPTH_IMAGE_TRANSPORT_ROS_VERSION_H_
