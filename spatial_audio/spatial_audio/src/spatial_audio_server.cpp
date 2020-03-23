// Standaerd C++ Library
#include <iostream>
#include <list>
#include <mutex>
#include <vector>
// Boost
#include <boost/shared_ptr.hpp>
// ROS
#include <audio_stream_msgs/AudioData.h>
#include <audio_stream_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <spatial_audio_msgs/PlaySpatialAudio.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
// USER
#include <spatial_audio/spatial_audio_source.h>
#include <spatial_audio/spatial_audio_server.h>
#include <spatial_audio/util.h>

namespace spatial_audio {

SpatialAudioServer::SpatialAudioServer( ros::NodeHandle& nh,
                                        ros::NodeHandle& nh_private,
                                        tf2_ros::Buffer& tf_buffer )
                : nh_(nh), nh_private_(nh_private), tf_buffer_(tf_buffer)
{
    // ROS params
    int num_spinthread;
    this->nh_private_.param<std::string>( "head_frame_id", this->head_frame_id_, "head_link" );
    this->nh_private_.param<int>( "num_spinthread", num_spinthread, 5 );

    // Service
    this->srv_= this->nh_private_.advertiseService<
                      SpatialAudioServer,
                      spatial_audio_msgs::PlaySpatialAudio::Request,
                      spatial_audio_msgs::PlaySpatialAudio::Response>( 
                  std::string("play_source"), 
                  &SpatialAudioServer::handlerPlayService,
                  this );

    // OpenAL
    // Opening an device
    const ALCchar* devicename = alcGetString( NULL, ALC_DEFAULT_DEVICE_SPECIFIER );
    this->device_ = alcOpenDevice( devicename );
    if ( not this->device_ ) {
        ROS_ERROR( "Unable to open default device" );
    }
    // create a context
    this->context_ = alcCreateContext( this->device_, NULL );
    if ( not alcMakeContextCurrent( this->context_ ) ) {
        ROS_ERROR( "Failed to make a default context" );
    } else if ( alGetError() != AL_NO_ERROR ) {
        ROS_ERROR( "something wrong happpened." );
    } else {
        ROS_INFO( "Created a default context." );
    }
    // setting up listener pose
    alcSuspendContext( this->context_ );
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
        alListenerfv( AL_POSITION, pos_listener );
        alListenerfv( AL_ORIENTATION, ori_listener );
    }
    alcProcessContext( this->context_ );
    // add spinner
    this->ptr_spinner_ = boost::shared_ptr<ros::AsyncSpinner>( new ros::AsyncSpinner(num_spinthread) );
}

SpatialAudioServer::~SpatialAudioServer()
{
    /* cleaning up ROS */
    this->ptr_spinner_->stop();
    /* delete all the audio source objects */
    while ( not this->list_audio_source_.empty() ) {
        this->delSource( this->list_audio_source_.front().getAudioSourceID() );
    }
    /* cleaning up OpenAL */
    alcMakeContextCurrent( NULL );
    alcDestroyContext( context_ );
    alcCloseDevice( device_ );
}

void SpatialAudioServer::spin( int spin_rate )
{
    // start callback spinner
    this->ptr_spinner_->start();

    // start main loop
    ros::Rate r( spin_rate );
    while ( ros::ok() ) {
        std::vector<int> vector_finished_id;
        mtx_audio_source_.lock();
        for ( auto itr = this->list_audio_source_.begin(); itr != this->list_audio_source_.end(); itr++  ) {
            ALint source_state = itr->getSourceState();
            if ( source_state != AL_PLAYING ) {
                vector_finished_id.push_back( itr->getAudioSourceID() );
            }
        }
        for ( int i = 0; i < vector_finished_id.size(); i++ ) {
            this->delSource( vector_finished_id[i] );
        }
        this->updateCoordinates();
        ROS_INFO( "Spinning, Finished: %ld, Playing: %ld",
                    vector_finished_id.size(),
                    this->list_audio_source_.size() - vector_finished_id.size() );
        mtx_audio_source_.unlock();
        r.sleep();
    }
}


bool SpatialAudioServer::handlerPlayService( 
        spatial_audio_msgs::PlaySpatialAudio::Request &req,
        spatial_audio_msgs::PlaySpatialAudio::Response &res )
{
    bool ret;
    switch ( req.action ) {

        case spatial_audio_msgs::PlaySpatialAudio::Request::ADD:

            ROS_INFO("ADD action recieved");

            this->mtx_audio_source_.lock();
            this->delSource( req.id ); // delete a SpatialAudioSource object if there is an one with the same id
            ret = this->addSource( req ); // add an audio source
            this->mtx_audio_source_.unlock();

            res.is_success = ret;

            break;

        case spatial_audio_msgs::PlaySpatialAudio::Request::DELETE:

            ROS_INFO("DELETE action recieved");

            this->mtx_audio_source_.lock();
            this->delSource( req.id );
            this->mtx_audio_source_.unlock();

            ret = true;
            res.is_success = true;

            break;

        case spatial_audio_msgs::PlaySpatialAudio::Request::DELETEALL:

            ROS_INFO("DELETEALL action recieved");

            this->mtx_audio_source_.lock();
            while ( not this->list_audio_source_.empty() ) {
                this->delSource( this->list_audio_source_.front().getAudioSourceID() );
            }
            this->mtx_audio_source_.unlock();

            ret = true;
            res.is_success = true;

            break;

        default:

            ROS_ERROR("Unknown action recieved.");

            ret = false;
            res.is_success = false;

            break;
    }
    return ret;
}

bool SpatialAudioServer::addSource( spatial_audio_msgs::PlaySpatialAudio::Request &req )
{
    auto itr = this->list_audio_source_.emplace( this->list_audio_source_.begin() );
    bool ret = itr->init( this->nh_, req );
    return ret;
}

bool SpatialAudioServer::delSource( int id )
{
    auto itr = this->findSource( id );
    if ( itr != this->list_audio_source_.end() ) {
        itr->close();
        this->list_audio_source_.erase( itr );
    }
    return true;
}

std::list<SpatialAudioSource>::iterator SpatialAudioServer::findSource( int id )
{
    std::list<SpatialAudioSource>::iterator itr =
        std::find_if( this->list_audio_source_.begin(),
                      this->list_audio_source_.end(), 
                      [&id]( SpatialAudioSource &x ){ return x.getAudioSourceID() == id; } );
    return itr;
}

void SpatialAudioServer::updateCoordinates()
{
    /* Update transforms */
    for ( auto itr = this->list_audio_source_.begin(); itr != this->list_audio_source_.end(); itr++  ) {
        itr->updateCoordinate( this->head_frame_id_, this->tf_buffer_, this->context_ );
    }
}

}
