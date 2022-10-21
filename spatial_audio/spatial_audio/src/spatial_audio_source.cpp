// Standaerd C++ Library
#include <iostream>
// Boost
#include <boost/shared_ptr.hpp>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
// ROS
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// ROS Messages
#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <spatial_audio_msgs/AudioSource.h>
// User
#include <spatial_audio/spatial_audio_source.h>
#include <spatial_audio/util.h>

namespace spatial_audio
{
SpatialAudioSource::SpatialAudioSource()
{
  this->initialized_ = false;
  this->playing_ = false;
}

SpatialAudioSource::SpatialAudioSource(ros::NodeHandle& nh, int audio_source_id, std::string source_frame_id,
                                       geometry_msgs::Pose source_pose, std::string stream_topic_audio,
                                       std::string stream_topic_info, double timeout)
{
  this->initialized_ = false;
  this->playing_ = false;
  if (not this->init(nh, audio_source_id, source_frame_id, source_pose, stream_topic_audio, stream_topic_info, timeout))
  {
    this->initialized_ = false;
  }
  else
  {
    this->initialized_ = true;
  }
}

SpatialAudioSource::~SpatialAudioSource()
{
  // stop playing
  if (this->initialized_)
  {
    this->close();
  }
}

bool SpatialAudioSource::init(ros::NodeHandle& nh, int audio_source_id, std::string source_frame_id,
                              geometry_msgs::Pose source_pose, std::string stream_topic_audio,
                              std::string stream_topic_info, double timeout)
{
  bool ret = true;

  // すでに initializa 済みであればエラーを返す
  if (this->initialized_)
  {
    return false;
  }

  // initialization
  /**
   * initialization of members
   */
  this->audio_source_id_ = audio_source_id;
  this->source_frame_id_ = source_frame_id;
  this->source_pose_ = source_pose;
  this->stream_topic_audio_ = stream_topic_audio;
  this->stream_topic_info_ = stream_topic_info;
  /**
   * Get a audio info message for meta information of audio stream
   */
  audio_common_msgs::AudioInfo::ConstPtr info =
      ros::topic::waitForMessage<audio_common_msgs::AudioInfo>(stream_topic_info, ros::Duration(timeout));
  if (not info)
  {
    ROS_ERROR("Cannot retrive a message from info topic: %s", stream_topic_info.c_str());
    return false;
  }
  if (info->channels == 1 && info->sample_format == "S8" )
  {
    this->stream_format_ = AL_FORMAT_MONO8;
  }
  else if (info->channels == 1 && info->sample_format == "S16LE" )
  {
    this->stream_format_ = AL_FORMAT_MONO16;
  }
  else if (info->channels == 2 && info->sample_format == "S8" )
  {
    this->stream_format_ = AL_FORMAT_STEREO8;
  }
  else if (info->channels == 2 && info->sample_format == "S16LE" )
  {
    this->stream_format_ = AL_FORMAT_STEREO16;
  }
  else
  {
    ROS_ERROR("Invalid channels or bit per sample in rosservice.");
    return false;
  }
  this->stream_sampling_rate_ = info->sample_rate;
  /**
   * generating OpenAL resources
   */
  // generate a source object
  alGenSources((ALuint)1, &this->al_source_id_);
  alSourcef(this->al_source_id_, AL_PITCH, 1);
  alSourcef(this->al_source_id_, AL_GAIN, 1);
  // AL_LOOPING should be AL_FALSE with stream source.
  // referece: https://stackoverflow.com/questions/6990701/why-would-alsourceunqueuebuffers-fail-with-invalid-operation
  alSourcei(this->al_source_id_, AL_LOOPING, AL_FALSE);
  /**
   * Subscribe the audio stream topic
   */
  this->stream_subscriber_ =
      nh.subscribe<audio_common_msgs::AudioData>(stream_topic_audio, 1, &SpatialAudioSource::callbackAudioStream, this);
  /**
   * Debug print
   */
  ROS_DEBUG("Add an audio source object. id: %d, frame_id: %s, stream topic: %s", this->audio_source_id_,
            this->source_frame_id_.c_str(), stream_topic_audio.c_str());
  /**
   * return
   */
  if (ret == true)
  {
    this->initialized_ = true;
  }
  return ret;
}

void SpatialAudioSource::close()
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->stream_subscriber_.shutdown();
  if (this->getSourceState() != AL_PLAYING)
  {
    alSourceStop(this->al_source_id_);
  }
  // release buffer object
  this->dequeALBuffers();
  // release source obect
  alDeleteSources(1, &this->al_source_id_);
  this->initialized_ = false;
}

void SpatialAudioSource::verbose()
{
  ROS_INFO_STREAM("audio source id:" << this->audio_source_id_ << " initialized:" << this->initialized_
                                     << " playing:" << this->playing_);
}

void SpatialAudioSource::update(std::string& source_frame_id, geometry_msgs::Pose& source_pose,
                                std::string& stream_topic_audio, std::string& stream_topic_info)
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->source_frame_id_ = source_frame_id;
  this->source_pose_ = source_pose;
  // TODO: update process for ros topic
}

void SpatialAudioSource::updateCoordinate(std::string& head_frame_id, tf2_ros::Buffer& tf_buffer, ALCcontext* context)
{
  geometry_msgs::TransformStamped transform_reference2head;
  geometry_msgs::Pose pose_source;
  std::string source_frame_id = this->source_frame_id_;
  geometry_msgs::Pose source_pose = this->source_pose_;
  try
  {
    transform_reference2head = tf_buffer.lookupTransform(head_frame_id.c_str(), source_frame_id.c_str(), ros::Time(0));
  }
  catch (const tf2::LookupException& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
  catch (const tf2::ExtrapolationException& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
  tf2::doTransform(source_pose, pose_source, transform_reference2head);

  alcSuspendContext(context);
  {
    std::lock_guard<std::mutex> lock(this->mtx_);
    ALfloat pos_source[3];
    pos_source[0] = pose_source.position.x;
    pos_source[1] = pose_source.position.y;
    pos_source[2] = pose_source.position.z;
    alSourcefv(this->al_source_id_, AL_POSITION, pos_source);
  }
  alcProcessContext(context);
}

bool SpatialAudioSource::isPlaying()
{
  return playing_;
}

void SpatialAudioSource::startSourcePlay(bool buffering, int buffer_num)
{
  this->playing_ = true;
  if (buffering)
  {
    this->waitBuffering(buffer_num);
  }
  alSourcePlay(this->al_source_id_);
}

void SpatialAudioSource::stopSourcePlay()
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  alSourceStop(this->al_source_id_);
  this->playing_ = false;
}

int SpatialAudioSource::getAudioSourceID()
{
  return this->audio_source_id_;
}

spatial_audio_msgs::AudioSource SpatialAudioSource::convertToROSMsg()
{
  spatial_audio_msgs::AudioSource msg;
  msg.source_frame_id = this->source_frame_id_;
  msg.audio_source_id = this->audio_source_id_;
  msg.source_pose = this->source_pose_;
  msg.stream_topic_audio = stream_topic_audio_;
  msg.stream_topic_info = stream_topic_info_;
  return msg;
}

void SpatialAudioSource::callbackAudioStream(const boost::shared_ptr<audio_common_msgs::AudioData const>& ptr_msg)
{
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->dequeALBuffers();

  // enqueue a new buffer with the received data
  if (this->playing_)
  {
    ALuint buffer_id;
    ALsizei buffer_size = ptr_msg->data.size();
    genBufferFromPCM(buffer_id, (ALvoid*)ptr_msg->data.data(), buffer_size, this->stream_sampling_rate_,
                     this->stream_format_);
    alSourceQueueBuffers(this->al_source_id_, 1, &buffer_id);

    ROS_DEBUG("New data containes %d bytes.", buffer_size);
  }

  if (this->playing_ and this->initialized_ and this->getSourceState() != AL_PLAYING)
  {
    alSourcePlay(this->al_source_id_);
  }
}

void SpatialAudioSource::waitBuffering(int buffer_num)
{
  /**
   * Wait until buffering
   */
  this->playing_ = true;
  ALsizei n = 0;
  while (n < buffer_num)
  {
    ros::Duration(1.0).sleep();
    alGetSourcei(this->al_source_id_, AL_BUFFERS_QUEUED, &n);
    ROS_DEBUG("waiting for buffering until %d buffers for id: %d with topic: %s, current buffers: %d...", buffer_num,
              this->audio_source_id_, this->stream_topic_audio_.c_str(), n);
  }
}

void SpatialAudioSource::dequeALBuffers()
{
  ALsizei n, m;
  alGetSourcei(this->al_source_id_, AL_BUFFERS_PROCESSED, &n);  // get a number of buffers processed.
  ALuint* buffers = new ALuint[n];  // These buffers should be released. Maybe shared_ptr should be used.
  alSourceUnqueueBuffers(this->al_source_id_, n, buffers);  // unqueue the buffers
  alDeleteBuffers(n, buffers);
  delete[] buffers;
  alGetSourcei(this->al_source_id_, AL_BUFFERS_QUEUED, &m);

  ROS_DEBUG("id:%d, %d buffers processed, %d buffers remains.", this->audio_source_id_, n, m);
}

ALint SpatialAudioSource::getSourceState()
{
  ALint source_state;
  alGetSourcei(this->al_source_id_, AL_SOURCE_STATE, &source_state);
  return source_state;
}

}  // namespace spatial_audio
