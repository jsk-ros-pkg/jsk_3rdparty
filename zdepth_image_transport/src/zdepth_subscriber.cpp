// #include <cv_bridge/cv_bridge.h>

#include <chrono>
#include "zdepth_image_transport/ros_version.h"
#if ROS_VERSION_MAJOR == 1
#include <ros/console.h>
#else
#include <rclcpp/logging.hpp>
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("zdepth_image_transport"), __VA_ARGS__)
#endif
#include "zdepth/zdepth.hpp"
#include "zdepth_image_transport/zdepth_subscriber.h"

#if ROS_VERSION_MAJOR == 1
#include <sensor_msgs/Image.h>
#include "zdepth_image_transport/ZDepthImage.h"
#else
#include <sensor_msgs/msg/image.hpp>
#include "zdepth_image_transport/msg/z_depth_image.hpp"
#endif

namespace zdepth_image_transport
{
#if ROS_VERSION_MAJOR == 1
  void ZDepthSubscriber::internalCallback(const zdepth_image_transport::ZDepthImageConstPtr& zdepth_msg,
                                          const Callback& user_cb)
#else
  void ZDepthSubscriber::internalCallback(const zdepth_image_transport::msg::ZDepthImage::ConstSharedPtr& zdepth_msg,
                                          const Callback& user_cb)
#endif
  {
    if (zdepth_msg->format != "16UC1; zdepth")
    {
      ROS_ERROR("Unsupported format: %s", zdepth_msg->format.c_str());
      return;
    }

    zdepth::DepthCompressor decompressor;
    std::vector<uint8_t> decompressed;
    int width = zdepth_msg->width;
    int height = zdepth_msg->height;

    auto start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::vector<uint16_t> depth;
    zdepth::DepthResult result = decompressor.Decompress(
      zdepth_msg->data, width, height, depth
    );

    auto decompress_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (result != zdepth::DepthResult::Success) {
      ROS_ERROR("Failed to decompress");
      return;
    }

    uint8_t* depth_vec_beg = reinterpret_cast<uint8_t *>(&depth[0]);
    uint8_t* depth_vec_end = depth_vec_beg + depth.size() * 2;
    std::vector<uint8_t> depth_vec(depth_vec_beg, depth_vec_end);
    auto copy_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    // DEBUG
    // ROS_DEBUG_STREAM("decompress: " << decompress_time - start_time << "ms");
    // ROS_DEBUG_STREAM("copy      : " << copy_time - decompress_time << "ms");

    // callback
#if ROS_VERSION_MAJOR == 1
    sensor_msgs::Image::Ptr img_msg(new sensor_msgs::Image());
#else
    sensor_msgs::msg::Image::SharedPtr img_msg(new sensor_msgs::msg::Image());
#endif
    img_msg->header = zdepth_msg->header;
    img_msg->data = depth_vec;
    img_msg->width = width;
    img_msg->height = height;
    img_msg->encoding = "16UC1";
    img_msg->step = width * 2;
    user_cb(img_msg);
  }

} //namespace zdepth_image_transport
