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
    zdepth_image_transport::ZDepthImage::Ptr compressed_img_msg(new zdepth_image_transport::ZDepthImage());
    compressed_img_msg->header = img_msg.header;
    compressed_img_msg->data = compressed;
    compressed_img_msg->format = "16UC1; zdepth";
    compressed_img_msg->width = width;
    compressed_img_msg->height = height;
    publish_fn(*compressed_img_msg);
  }

} //namespace zdepth_image_transport
