// #include <cv_bridge/cv_bridge.h>

#include <ros/console.h>
#include "zdepth/zdepth.hpp"
#include "zdepth_image_transport/zdepth_publisher.h"

#include <sensor_msgs/Image.h>
#include "zdepth_image_transport/ZDepthImage.h"

namespace zdepth_image_transport
{

  void ZDepthPublisher::publish(const sensor_msgs::Image& img_msg,
                                const PublishFn& publish_fn) const
  {
    if (img_msg.encoding != "16UC1")
    {
      ROS_ERROR("Unsupported encoding: %s", img_msg.encoding.c_str());
      return;
    }

    zdepth::DepthCompressor compressor;
    std::vector<uint8_t> compressed;
    int width = img_msg.width;
    int height = img_msg.height;

    const uint16_t* frame = (uint16_t*)img_msg.data.data();
    zdepth::DepthResult result = compressor.Compress(width, height, frame, compressed, true);
    if (result != zdepth::DepthResult::Success) {
      ROS_ERROR("Failed to compress");
      return;
    }

    // publish
    zdepth_image_transport::ZDepthImage::Ptr compressed_img_msg(new zdepth_image_transport::ZDepthImage());
    compressed_img_msg->header = img_msg.header;
    compressed_img_msg->data = compressed;
    compressed_img_msg->format = "16UC1; zdepth";
    compressed_img_msg->width = width;
    compressed_img_msg->height = height;
    publish_fn(*compressed_img_msg);
  }

} //namespace zdepth_image_transport
