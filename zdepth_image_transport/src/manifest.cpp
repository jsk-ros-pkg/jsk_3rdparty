#include <pluginlib/class_list_macros.h>
#include <zdepth_image_transport/zdepth_publisher.h>
#include <zdepth_image_transport/zdepth_subscriber.h>

PLUGINLIB_EXPORT_CLASS(zdepth_image_transport::ZDepthPublisher, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(zdepth_image_transport::ZDepthSubscriber, image_transport::SubscriberPlugin)
