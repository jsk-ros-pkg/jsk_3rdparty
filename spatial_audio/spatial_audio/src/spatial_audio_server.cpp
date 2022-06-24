// Standaerd C++ Library
#include <iostream>
#include <list>
#include <mutex>
#include <vector>
#include <optional>
// Boost
#include <boost/shared_ptr.hpp>
// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// ROS Messages and Services
#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <spatial_audio_msgs/AudioSource.h>
#include <spatial_audio_msgs/AudioSourceArray.h>
#include <spatial_audio_msgs/AddSpatialAudio.h>
#include <spatial_audio_msgs/UpdateSpatialAudio.h>
#include <spatial_audio_msgs/TriggerSpatialAudio.h>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
// USER
#include <spatial_audio/spatial_audio_server.h>
#include <spatial_audio/spatial_audio_source.h>
#include <spatial_audio/util.h>

namespace spatial_audio
{
SpatialAudioServer::SpatialAudioServer(ros::NodeHandle& nh, ros::NodeHandle& nh_private, tf2_ros::Buffer& tf_buffer)
  : nh_(nh), nh_private_(nh_private), tf_buffer_(tf_buffer)
{
  // ROS params
  int num_spinthread;
  this->nh_private_.param<std::string>("head_frame_id", this->head_frame_id_, "head_link");
  this->nh_private_.param<int>("num_spinthread", num_spinthread, 5);
  this->nh_private_.param<std::string>("hrtfname", this->hrtfname_, "");

  // Service
  this->srv_add_ = this->nh_private_.advertiseService<SpatialAudioServer, spatial_audio_msgs::AddSpatialAudio::Request,
                                                      spatial_audio_msgs::AddSpatialAudio::Response>(
      std::string("add_audio_source"), &SpatialAudioServer::handlerAddService, this);
  this->srv_update_ =
      this->nh_private_.advertiseService<SpatialAudioServer, spatial_audio_msgs::UpdateSpatialAudio::Request,
                                         spatial_audio_msgs::UpdateSpatialAudio::Response>(
          std::string("update_audio_source"), &SpatialAudioServer::handlerUpdateService, this);
  this->srv_remove_ =
      this->nh_private_.advertiseService<SpatialAudioServer, spatial_audio_msgs::TriggerSpatialAudio::Request,
                                         spatial_audio_msgs::TriggerSpatialAudio::Response>(
          std::string("remove_audio_source"), &SpatialAudioServer::handlerRemoveService, this);
  this->srv_remove_all_ =
      this->nh_private_.advertiseService<SpatialAudioServer, std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          std::string("remove_all_audio_source"), &SpatialAudioServer::handlerRemoveAllService, this);
  this->srv_play_ =
      this->nh_private_.advertiseService<SpatialAudioServer, spatial_audio_msgs::TriggerSpatialAudio::Request,
                                         spatial_audio_msgs::TriggerSpatialAudio::Response>(
          std::string("play_audio_source"), &SpatialAudioServer::handlerPlayService, this);
  this->srv_stop_ =
      this->nh_private_.advertiseService<SpatialAudioServer, spatial_audio_msgs::TriggerSpatialAudio::Request,
                                         spatial_audio_msgs::TriggerSpatialAudio::Response>(
          std::string("stop_audio_source"), &SpatialAudioServer::handlerStopService, this);

  // Publisher
  this->pub_audio_source_array_ =
      this->nh_private_.advertise<spatial_audio_msgs::AudioSourceArray>(std::string("audio_source_array"), 1);

  // OpenAL
  // Opening an device
  const ALCchar* devicename = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
  this->device_ = alcOpenDevice(devicename);
  if (not this->device_)
  {
    ROS_ERROR("Unable to open default device");
  }
  // create a context
  this->context_ = alcCreateContext(this->device_, NULL);
  if (not alcMakeContextCurrent(this->context_))
  {
    ROS_ERROR("Failed to make a default context");
  }
  else if (alGetError() != AL_NO_ERROR)
  {
    ROS_ERROR("something wrong happpened.");
  }
  else
  {
    ROS_INFO("Created a default context.");
  }

  // setting up listener pose
  alcSuspendContext(this->context_);
  {
    ALfloat pos_listener[3];
    ALfloat ori_listener[6];
    pos_listener[0] = 0;
    pos_listener[1] = 0;
    pos_listener[2] = 0;
    ori_listener[0] = 1;
    ori_listener[1] = 0;
    ori_listener[2] = 0;
    ori_listener[3] = 0;
    ori_listener[4] = 0;
    ori_listener[5] = 1;
    alListenerfv(AL_POSITION, pos_listener);
    alListenerfv(AL_ORIENTATION, ori_listener);
  }
  alcProcessContext(this->context_);
  // add spinner
  this->ptr_spinner_ = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(num_spinthread));
}

SpatialAudioServer::~SpatialAudioServer()
{
  /* cleaning up ROS */
  this->ptr_spinner_->stop();
  /* delete all the audio source objects */
  while (not this->list_audio_source_.empty())
  {
    this->removeAudioSource(this->list_audio_source_.front().getAudioSourceID());
  }
  /* cleaning up OpenAL */
  alcMakeContextCurrent(NULL);
  alcDestroyContext(context_);
  alcCloseDevice(device_);
}

void SpatialAudioServer::spin(int spin_rate)
{
  // Prepairing alsoft configuration with the given hrtf file for OpenAL
  {
    if (!alcIsExtensionPresent(device_, "ALC_SOFT_HRTF"))
    {
      ROS_ERROR("ALC_SOFT_HRTF not supported.");
      return;
    }

    ALCint num_hrtf;
    alcGetIntegerv(device_, ALC_NUM_HRTF_SPECIFIERS_SOFT, 1, &num_hrtf);
    if (!num_hrtf)
    {
      ROS_ERROR("No HRTFs found.\n");
    }
    else
    {
      int index = -1;
      ROS_INFO("Available HRTFs:\n");
      for (int i = 0; i < num_hrtf; i++)
      {
        const ALCchar* name = alcGetStringiSOFT(device_, ALC_HRTF_SPECIFIER_SOFT, i);
        ROS_INFO("    %d: %s\n", i, name);
        if (hrtfname_ == name)
        {
          index = i;
        }
      }

      ALCint attr[5];
      if (hrtfname_.empty() or index == -1)
      {
        ROS_INFO("Specified HRTF not found or not specified. using default hrtf...");
        attr[0] = ALC_HRTF_SOFT;
        attr[1] = ALC_TRUE;
        attr[2] = 0;
      }
      else
      {
        ROS_INFO("Specified HRTF found.");
        attr[0] = ALC_HRTF_SOFT;
        attr[1] = ALC_TRUE;
        attr[2] = ALC_HRTF_ID_SOFT;
        attr[3] = index;
        attr[4] = 0;
      }

      if (!alcResetDeviceSOFT(device_, attr))
      {
        ROS_ERROR("Failed to reset device: %s", alcGetString(device_, alcGetError(device_)));
        return;
      }
    }

    ALCint hrtf_state;
    alcGetIntegerv(device_, ALC_HRTF_SOFT, 1, &hrtf_state);
    if (!hrtf_state)
    {
      ROS_ERROR("HRTF not enabled!");
      return;
    }
    else
    {
      const ALchar* name = alcGetString(device_, ALC_HRTF_SPECIFIER_SOFT);
      ROS_INFO("HRTF enabled, using %s", name);
    }
  }

  // start callback spinner
  this->ptr_spinner_->start();

  ROS_INFO("spatial_audio_server started.");

  // start main loop
  ros::Rate r(spin_rate);
  while (ros::ok())
  {
    this->mtx_audio_source_.lock();
    /*
    // TODO: timeout したもののみを vector_finished に入れる
    std::vector<int> vector_finished_id;
    for ( auto itr = this->list_audio_source_.begin(); itr != this->list_audio_source_.end(); itr++  ) {
        ALint source_state = itr->getSourceState();
        if ( source_state != AL_PLAYING ) {
            vector_finished_id.push_back( itr->getAudioSourceID() );
        }
    }
    for ( int i = 0; i < vector_finished_id.size(); i++ ) {
        this->delSource( vector_finished_id[i] );
    }
    ROS_INFO( "Spinning, Finished: %ld, Playing: %ld",
                vector_finished_id.size(),
                this->list_audio_source_.size() - vector_finished_id.size() );
    */
    std::vector<int> vector_stopped_id;
    std::vector<int> vector_playing_id;
    this->updateCoordinates();
    this->mtx_audio_source_.unlock();
    r.sleep();
  }
}

bool SpatialAudioServer::handlerAddService(spatial_audio_msgs::AddSpatialAudio::Request& req,
                                           spatial_audio_msgs::AddSpatialAudio::Response& res)
{
  ROS_DEBUG_STREAM("add called." << req);
  std::lock_guard<std::mutex> lock(this->mtx_audio_source_);
  int audio_source_id = this->getNewSourceID();
  auto result = this->addAudioSource(audio_source_id, req.audio_source.source_frame_id, req.audio_source.source_pose,
                                     req.audio_source.stream_topic_audio, req.audio_source.stream_topic_info);
  if (not result)
  {
    this->removeAudioSource(audio_source_id);
    res.success = true;
    res.message = std::string("Failed to add audio source");
    ROS_DEBUG_STREAM("add finished." << res);
    return true;
  }

  if (req.auto_play)
  {
    auto itr = result.value();
    itr->startSourcePlay();
  }
  res.success = true;
  res.audio_source_id = audio_source_id;
  ROS_DEBUG_STREAM("add finished." << res);
  return true;
}

bool SpatialAudioServer::handlerUpdateService(spatial_audio_msgs::UpdateSpatialAudio::Request& req,
                                              spatial_audio_msgs::UpdateSpatialAudio::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mtx_audio_source_);
  auto result =
      this->updateAudioSource(req.audio_source_id, req.audio_source.source_frame_id, req.audio_source.source_pose,
                              req.audio_source.stream_topic_audio, req.audio_source.stream_topic_info);
  if (result)
  {
    res.success = true;
  }
  else
  {
    res.success = false;
    res.message = std::string("Failed to update audio source");
  }
  return true;
}

bool SpatialAudioServer::handlerRemoveService(spatial_audio_msgs::TriggerSpatialAudio::Request& req,
                                              spatial_audio_msgs::TriggerSpatialAudio::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mtx_audio_source_);
  bool success = this->removeAudioSource(req.audio_source_id);
  if (success)
  {
    res.success = success;
  }
  else
  {
    res.success = success;
    res.message = std::string("Failed to remove audio source");
  }
  return true;
}

bool SpatialAudioServer::handlerRemoveAllService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mtx_audio_source_);
  bool success = true;
  bool ret = true;
  while (ret and (not this->list_audio_source_.empty()))
  {
    ret = this->removeAudioSource(this->list_audio_source_.front().getAudioSourceID());
    success = ret and success;
  }

  if (success)
  {
    res.success = success;
  }
  else
  {
    res.success = success;
    res.message = std::string("Failed to remove all audio source");
  }
  return true;
}

bool SpatialAudioServer::handlerPlayService(spatial_audio_msgs::TriggerSpatialAudio::Request& req,
                                            spatial_audio_msgs::TriggerSpatialAudio::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mtx_audio_source_);
  auto result = this->findAudioSource(req.audio_source_id);
  if (not result)
  {
    res.success = false;
    res.message = "Failed to find audio source.";
    return true;
  }
  else
  {
    auto itr = result.value();
    itr->startSourcePlay();
    res.success = true;
    return true;
  }
}

bool SpatialAudioServer::handlerStopService(spatial_audio_msgs::TriggerSpatialAudio::Request& req,
                                            spatial_audio_msgs::TriggerSpatialAudio::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mtx_audio_source_);
  auto result = this->findAudioSource(req.audio_source_id);
  if (not result)
  {
    res.success = false;
    res.message = "Failed to find audio source.";
    return true;
  }
  else
  {
    auto itr = result.value();
    itr->stopSourcePlay();
    res.success = true;
    return true;
  }
}

std::optional<std::list<SpatialAudioSource>::iterator>
SpatialAudioServer::addAudioSource(int audio_source_id, std::string source_frame_id, geometry_msgs::Pose source_pose,
                                   std::string stream_topic_audio, std::string stream_topic_info)
{
  auto itr = this->list_audio_source_.emplace(this->list_audio_source_.begin());
  bool success =
      itr->init(this->nh_, audio_source_id, source_frame_id, source_pose, stream_topic_audio, stream_topic_info);
  if (success)
  {
    ROS_DEBUG_STREAM("Add an audio source.");
    return itr;
  }
  else
  {
    ROS_DEBUG_STREAM("Failed to add an audio source.");
    return std::nullopt;
  }
}

std::optional<std::list<SpatialAudioSource>::iterator>
SpatialAudioServer::updateAudioSource(int audio_source_id, std::string source_frame_id, geometry_msgs::Pose source_pose,
                                      std::string stream_topic_audio, std::string stream_topic_info)
{
  auto result = this->findAudioSource(audio_source_id);
  if (result)
  {
    auto itr = result.value();
    itr->update(source_frame_id, source_pose, stream_topic_audio, stream_topic_info);
    ROS_DEBUG_STREAM("Update an audio source.");
    return itr;
  }
  else
  {
    ROS_DEBUG_STREAM("Failed to update an audio source. id: " << audio_source_id);
    return std::nullopt;
  }
}

bool SpatialAudioServer::removeAudioSource(int audio_source_id)
{
  auto result = this->findAudioSource(audio_source_id);
  if (result)
  {
    auto itr = result.value();
    itr->close();
    this->list_audio_source_.erase(itr);
    return true;
  }
  else
  {
    return false;
  }
}

std::optional<std::list<SpatialAudioSource>::iterator> SpatialAudioServer::findAudioSource(int audio_source_id)
{
  std::list<SpatialAudioSource>::iterator itr =
      std::find_if(this->list_audio_source_.begin(), this->list_audio_source_.end(),
                   [&audio_source_id](SpatialAudioSource& x) { return x.getAudioSourceID() == audio_source_id; });
  if (itr == this->list_audio_source_.end())
  {
    return std::nullopt;
  }
  else
  {
    return itr;
  }
}

int SpatialAudioServer::getNewSourceID()
{
  auto itr_max_id = std::max_element(this->list_audio_source_.begin(), this->list_audio_source_.end(),
                                     [](auto& a, auto& b) { return a.getAudioSourceID() < b.getAudioSourceID(); });
  int audio_source_id;
  if (itr_max_id == this->list_audio_source_.end())
  {
    audio_source_id = 1;
  }
  else
  {
    audio_source_id = itr_max_id->getAudioSourceID() + 1;
  }
  return audio_source_id;
}

void SpatialAudioServer::updateCoordinates()
{
  /* Update transforms */
  for (auto itr = this->list_audio_source_.begin(); itr != this->list_audio_source_.end(); itr++)
  {
    itr->updateCoordinate(this->head_frame_id_, this->tf_buffer_, this->context_);
    itr->verbose();
  }
}

}  // namespace spatial_audio
