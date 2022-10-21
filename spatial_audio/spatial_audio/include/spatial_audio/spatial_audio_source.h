#ifndef SPATIAL_AUDIO_SPATIAL_AUDIO_SOURCE_H__
#define SPATIAL_AUDIO_SPATIAL_AUDIO_SOURCE_H__

// Standaerd C++ Library
#include <iostream>
#include <mutex>
// Boost
#include <boost/shared_ptr.hpp>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// ROS Messages
#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <spatial_audio_msgs/AudioSource.h>

namespace spatial_audio
{
class SpatialAudioSource
{
public:
  SpatialAudioSource();
  SpatialAudioSource(ros::NodeHandle& nh, int audio_source_id, std::string source_frame_id,
                     geometry_msgs::Pose source_pose, std::string stream_topic_audio, std::string stream_topic_info,
                     double timeout = 1.0);
  ~SpatialAudioSource();
  /**
   * non copyable settings due to mutex
   */
  SpatialAudioSource(const SpatialAudioSource&) = delete;
  SpatialAudioSource& operator=(const SpatialAudioSource&) = delete;
  /**
   * non copyable settings due to mutex
   */
  SpatialAudioSource(SpatialAudioSource&&) = delete;
  SpatialAudioSource& operator=(SpatialAudioSource&&) = delete;
  /**
   * Initialzation function. This function must be called before an object is actually used.
   * @param[in] nh ros node handler
   * @param[in] req a request of an audio source to add
   */
  bool init(ros::NodeHandle& nh, int audio_source_id, std::string source_frame_id, geometry_msgs::Pose source_pose,
            std::string stream_topic_audio, std::string stream_topic_info, double timeout = 1.0);
  /**
   * Finalization function. This function should be called when an audio source is discarded.
   */
  void close();
  /**
   * print audio source information to ros logging system
   */
  void verbose();
  /**
   * update parameters of and audio source
   */
  void update(std::string& source_frame_id, geometry_msgs::Pose& source_pose, std::string& stream_topic_audio,
              std::string& stream_topic_info);
  /**
   * update the pose of an audio source
   * @param[in] head_frame_id frame_id of a head
   * @param[in] tf_buffer tf2_ros buffer
   * @param[in] context OpenAL context handler
   */
  void updateCoordinate(std::string& head_frame_id, tf2_ros::Buffer& tf_buffer, ALCcontext* context);
  /**
   * Return is the source is playing
   */
  bool isPlaying();
  /**
   * Start playing the source
   *   Race condition is not considered
   */
  void startSourcePlay(bool buffering = true, int buffer_num = 1);
  /**
   * Stop playing the source
   */
  void stopSourcePlay();
  /**
   * Get Audio Source ID
   */
  int getAudioSourceID();
  /**
   * @brief Generate spatial_audio_msgs/AudioSource instance
   *
   */
  spatial_audio_msgs::AudioSource convertToROSMsg();

private:
  /**
   * ROS callback function for buffering stream audio
   */
  void callbackAudioStream(const boost::shared_ptr<audio_common_msgs::AudioData const>& ptr_msg);
  /**
   * deque OpenAL buffers done
   *   Race condition is not considered
   */
  void dequeALBuffers();
  /**
   * @brief Wait for buffering.
   *   Race condition is not considered
   */
  void waitBuffering(int buffer_num = 1);
  /**
   * Return OpenAL state of the source
   *   Race condition is not considered
   */
  ALint getSourceState();

  int audio_source_id_;  // audio source id
  std::mutex mtx_;       // mutex for controling access to the resource
  bool initialized_;     // flag for initialization
  bool playing_;         // flag for playing
  /*
   * ROS related variables
   */
  std::string source_frame_id_;      // the frame_id to which this source is fixed.
  geometry_msgs::Pose source_pose_;  // the pose from the origin of source_frame_id to source
  /*
   * OpenAL related variables
   */
  ALuint al_source_id_;  // source id for OpenAL source object
  /**
   * Audio stream related variables
   */
  std::string stream_topic_audio_;
  std::string stream_topic_info_;
  ros::Subscriber stream_subscriber_;
  ALsizei stream_sampling_rate_;
  ALenum stream_format_;
};
}  // namespace spatial_audio

#endif
