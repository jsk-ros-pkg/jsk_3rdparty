#ifndef SPATIAL_AUDIO_SPATIAL_AUDIO_SERVER_H__
#define SPATIAL_AUDIO_SPATIAL_AUDIO_SERVER_H__

// Standaerd C++ Library
#include <iostream>
#include <list>
#include <mutex>
#include <string>
#include <optional>
// Boost
#include <boost/shared_ptr.hpp>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
// ROS
#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <spatial_audio_msgs/AudioSourceArray.h>
#include <spatial_audio_msgs/AddSpatialAudio.h>
#include <spatial_audio_msgs/UpdateSpatialAudio.h>
#include <spatial_audio_msgs/TriggerSpatialAudio.h>
#include <std_srvs/Trigger.h>

// USER
#include <spatial_audio/spatial_audio_source.h>

namespace spatial_audio
{
class SpatialAudioServer
{
public:
  /**
   * Constructor
   * @param[in] nh ros node handler
   * @param[in] nh_private ros node handler with private namespace
   * @param[in] tf_buffer tf2_ros buffer
   */
  SpatialAudioServer(ros::NodeHandle& nh, ros::NodeHandle& nh_private, tf2_ros::Buffer& tf_buffer);
  ~SpatialAudioServer();

  /**
   * spin function
   * @param[in] spin_rate spin rate for main loop [hz]
   */
  void spin(int spin_rate);

private:
  /**
   * ROS serivice handlers
   */
  bool handlerAddService(spatial_audio_msgs::AddSpatialAudio::Request& req,
                         spatial_audio_msgs::AddSpatialAudio::Response& res);

  bool handlerUpdateService(spatial_audio_msgs::UpdateSpatialAudio::Request& req,
                            spatial_audio_msgs::UpdateSpatialAudio::Response& res);

  bool handlerRemoveService(spatial_audio_msgs::TriggerSpatialAudio::Request& req,
                            spatial_audio_msgs::TriggerSpatialAudio::Response& res);

  bool handlerRemoveAllService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool handlerPlayService(spatial_audio_msgs::TriggerSpatialAudio::Request& req,
                          spatial_audio_msgs::TriggerSpatialAudio::Response& res);

  bool handlerStopService(spatial_audio_msgs::TriggerSpatialAudio::Request& req,
                          spatial_audio_msgs::TriggerSpatialAudio::Response& res);

  /**
   * add an audio source
   * @param[in] req a request of an audio source to add
   */
  std::optional<std::list<SpatialAudioSource>::iterator> addAudioSource(int audio_source_id,
                                                                        std::string source_frame_id,
                                                                        geometry_msgs::Pose source_pose,
                                                                        std::string stream_topic_audio,
                                                                        std::string stream_topic_info);
  /**
   * update an audio source
   */
  std::optional<std::list<SpatialAudioSource>::iterator> updateAudioSource(int audio_source_id,
                                                                           std::string source_frame_id,
                                                                           geometry_msgs::Pose source_pose,
                                                                           std::string stream_topic_audio,
                                                                           std::string stream_topic_info);
  /**
   * delete an audio source
   * @param[in] id id number of an audio source to delete
   */
  bool removeAudioSource(int audio_source_id);
  /**
   * check if there is an audio source object with a specified id
   * @param[in] id id number of an audio source to check
   */
  std::optional<std::list<SpatialAudioSource>::iterator> findAudioSource(int audio_source_id);
  /**
   * @brief get a unique audio source id for new audio source.
   *
   */
  int getNewSourceID();
  /**
   * update transformation values from head frame to each source frame.
   */
  void updateCoordinates();

  /**
   * OpenAL resources
   */
  ALCcontext* context_;
  ALCdevice* device_;
  std::string hrtfname_;

  /**
   * ROS related resources
   */
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  tf2_ros::Buffer& tf_buffer_;
  ros::ServiceServer srv_add_;
  ros::ServiceServer srv_update_;
  ros::ServiceServer srv_remove_;
  ros::ServiceServer srv_remove_all_;
  ros::ServiceServer srv_play_;
  ros::ServiceServer srv_stop_;
  ros::Publisher pub_audio_source_array_;
  boost::shared_ptr<ros::AsyncSpinner> ptr_spinner_;
  std::string head_frame_id_;

  /**
   * audio source management related
   */
  std::list<SpatialAudioSource> list_audio_source_;
  std::mutex mtx_audio_source_;
};
}  // namespace spatial_audio

#endif
