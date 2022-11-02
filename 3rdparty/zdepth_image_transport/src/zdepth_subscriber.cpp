// #include <cv_bridge/cv_bridge.h>

#include <ros/console.h>
#include "zdepth/zdepth.hpp"
#include "zdepth_image_transport/zdepth_subscriber.h"

#include <sensor_msgs/Image.h>
#include "zdepth_image_transport/ZDepthImage.h"

namespace zdepth_image_transport
{
  void ZDepthSubscriber::internalCallback(const zdepth_image_transport::ZDepthImageConstPtr& zdepth_msg,
                                          const Callback& user_cb)
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

    std::vector<uint16_t> depth;
    zdepth::DepthResult result = decompressor.Decompress(
      zdepth_msg->data, width, height, depth
    );

    if (result != zdepth::DepthResult::Success) {
      ROS_ERROR("Failed to decompress");
      return;
    }

    std::vector<uint8_t> depth_vec;
    for (int i=0; i<depth.size(); ++i){
       uint8_t* d = reinterpret_cast<uint8_t *>(&depth[i]);
       std::vector<uint8_t> dv(d, d+2);
       depth_vec.insert(depth_vec.end(), dv.begin(), dv.end());
    }

    // callback
    sensor_msgs::Image::Ptr img_msg(new sensor_msgs::Image());
    img_msg->header = zdepth_msg->header;
    img_msg->data = depth_vec;
    img_msg->width = width;
    img_msg->height = height;
    img_msg->encoding = "16UC1";
    img_msg->step = width * 2;
    user_cb(img_msg);
  }

} //namespace zdepth_image_transport
