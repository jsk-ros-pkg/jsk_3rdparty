// #include <cv_bridge/cv_bridge.h>

#include "zdepth_image_transport/ros_version.h"
#if ROS_VERSION_MAJOR == 1
#include <ros/console.h>
#else
#include <rclcpp/logging.hpp>
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("zdepth_image_transport"), __VA_ARGS__)
#endif
#include "zdepth/zdepth.hpp"
#include "zdepth_image_transport/zdepth_publisher.h"

#if ROS_VERSION_MAJOR == 1
#include <sensor_msgs/Image.h>
#include "zdepth_image_transport/ZDepthImage.h"
#else
#include <cmath>
#include <sensor_msgs/msg/image.hpp>
#include "zdepth_image_transport/msg/z_depth_image.hpp"
#endif

namespace zdepth_image_transport
{

#if ROS_VERSION_MAJOR == 1
  void ZDepthPublisher::publish(const sensor_msgs::Image& img_msg,
                                const PublishFn& publish_fn) const
#else
  void ZDepthPublisher::publish(const sensor_msgs::msg::Image& img_msg,
                                const PublisherT& publisher) const
#endif
  {
    if (img_msg.encoding != "16UC1" && img_msg.encoding != "32FC1")
    {
      ROS_ERROR("Unsupported encoding: %s", img_msg.encoding.c_str());
      return;
    }

    zdepth::DepthCompressor compressor;
    std::vector<uint8_t> compressed;
    int width = img_msg.width;
    int height = img_msg.height;
    zdepth::DepthResult result;

    if (img_msg.encoding == "16UC1")
    {
      const uint16_t* frame = (uint16_t*)img_msg.data.data();
      result = compressor.Compress(width, height, frame, compressed, true);
    }
    else
    {
      // convert 32fc1 -> 16uc1
      const float* frame_32fc1 = reinterpret_cast<const float*>(img_msg.data.data());
      uint16_t* frame_16uc1 = new uint16_t[height*width]();
      for (unsigned index = 0; index < height * width; ++index)
      {
        float d = frame_32fc1[index];
        frame_16uc1[index] = std::isnan(d) ? 0 : (uint16_t)(d * 1000);
      }
      result = compressor.Compress(width, height, frame_16uc1, compressed, true);
    }

    if (result != zdepth::DepthResult::Success) {
      ROS_ERROR("Failed to compress");
      return;
    }

    // publish
#if ROS_VERSION_MAJOR == 1
    zdepth_image_transport::ZDepthImage::Ptr compressed_img_msg(new zdepth_image_transport::ZDepthImage());
#else
    zdepth_image_transport::msg::ZDepthImage::SharedPtr compressed_img_msg(new zdepth_image_transport::msg::ZDepthImage());
#endif
    compressed_img_msg->header = img_msg.header;
    compressed_img_msg->data = compressed;
    compressed_img_msg->format = "16UC1; zdepth";
    compressed_img_msg->width = width;
    compressed_img_msg->height = height;
#if ROS_VERSION_MAJOR == 1
    publish_fn(*compressed_img_msg);
#else
    publisher->publish(*compressed_img_msg);
#endif
  }

} //namespace zdepth_image_transport
