#include "zdepth_image_transport/ros_version.h"
#if ROS_VERSION_MAJOR == 1
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif
#include <zdepth_image_transport/zdepth_publisher.h>
#include <zdepth_image_transport/zdepth_subscriber.h>

PLUGINLIB_EXPORT_CLASS(zdepth_image_transport::ZDepthPublisher, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(zdepth_image_transport::ZDepthSubscriber, image_transport::SubscriberPlugin)
