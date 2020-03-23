// Standaerd C++ Library
#include <iostream>
// Boost
#include <boost/shared_ptr.hpp>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
// ROS
#include <audio_stream_msgs/AudioData.h>
#include <audio_stream_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <spatial_audio_msgs/PlaySpatialAudio.h>
#include <tf2_ros/transform_listener.h>
// User
#include <spatial_audio/spatial_audio_source.h>
#include <spatial_audio/util.h>

namespace spatial_audio {

SpatialAudioSource::SpatialAudioSource()
{
    this->is_init_ = false;
}

SpatialAudioSource::SpatialAudioSource(
        ros::NodeHandle& nh,
        spatial_audio_msgs::PlaySpatialAudio::Request& req )
{
    this->is_init_ = false;
    if ( not this->init(nh,req) ) {
        this->is_init_ = false;
    } else {
        this->is_init_ = true;
    }
}

SpatialAudioSource::~SpatialAudioSource()
{
    // stop playing
    if ( this->is_init_ ) {
        this->close();
    }
}

bool SpatialAudioSource::init( ros::NodeHandle& nh, spatial_audio_msgs::PlaySpatialAudio::Request &req )
{
    bool ret = true;

    // すでに initializa 済みであればエラーを返す
    if ( this->is_init_ ) {
        return false;
    }

    // req の action が ADD でなければ弾く. assert の方が良い?
    if ( req.action !=  spatial_audio_msgs::PlaySpatialAudio::Request::ADD ) {
        ROS_ERROR( "The action of a given request is not ADD." );
        ret = false;
    }

    // initialization
    /**
     * initialization of members
     */
    this->id_ = req.id;
    this->source_frame_id_ = req.header.frame_id;
    this->source_pose_ = req.pose;
    /**
     * Get a audio info message for meta information of audio stream
     */
    audio_stream_msgs::AudioInfo::ConstPtr info =
        ros::topic::waitForMessage<audio_stream_msgs::AudioInfo>( req.stream_topic_info );
    if ( not info ) {
        ROS_ERROR( "Cannot retrive a message from info topic: %s", req.stream_topic_info.c_str() );
        return false;
    }
    if ( info->channels == audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_MONAURAL
      && info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_8BIT ) {
        this->stream_format_ = AL_FORMAT_MONO8;
    } else if ( info->channels ==  audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_MONAURAL
             && info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_16BIT ) {
        this->stream_format_ = AL_FORMAT_MONO16;
    } else if ( info->channels ==  audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_STEREO
             && info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_8BIT ) {
        this->stream_format_ = AL_FORMAT_STEREO8;
    } else if ( info->channels ==  audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_STEREO
             && info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_16BIT ) {
        this->stream_format_ = AL_FORMAT_STEREO16;
    } else {
        ROS_ERROR("Invalid channels or bit per sample in rosservice.");
        return false;
    }
    this->stream_sampling_rate_ = info->sampling_rate;
    /**
     * generating OpenAL resources
     */
    // generate a source object
    alGenSources( (ALuint)1, &this->al_source_id_ );
    alSourcef( this->al_source_id_, AL_PITCH, 1 );
    alSourcef( this->al_source_id_, AL_GAIN, 1 );
    // AL_LOOPING should be AL_FALSE with stream source.
    // referece: https://stackoverflow.com/questions/6990701/why-would-alsourceunqueuebuffers-fail-with-invalid-operation
    alSourcei( this->al_source_id_, AL_LOOPING, AL_FALSE );
    /**
     * Subscribe the audio stream topic
     */
    this->stream_subscriber_ =
        nh.subscribe<audio_stream_msgs::AudioData>(
                req.stream_topic_audio,
                1000,
                &SpatialAudioSource::callbackAudioStream,
                this );
    /**
     * Wait until buffering
     */
    ALsizei n = 0;
    while ( n < 10 ) {
        ros::Duration( 1.0 ).sleep();
        alGetSourcei( this->al_source_id_, AL_BUFFERS_QUEUED, &n );
        ROS_INFO( "waiting for buffering..." );
    }
    /**
     * start playing
     */
    alSourcePlay( this->al_source_id_ );
    /**
     * Debug print
     */
    ROS_INFO( "Add an audio source object. id: %d, frame_id: %s, stream topic: %s", this->id_, this->source_frame_id_.c_str(), req.stream_topic_audio.c_str() );
    /**
     * return
     */
    if ( ret == true ) {
        this->is_init_ = true;
    }
    return ret;
}

void SpatialAudioSource::close()
{
    this->mtx_.lock();
        this->stream_subscriber_.shutdown();
        alSourceStop( this->al_source_id_ );
    this->mtx_.unlock();
    // release buffer object
    this->dequeALBuffers();
    // release source obect
    alDeleteSources( 1, &this->al_source_id_ );
    this->is_init_ = false;
}

void SpatialAudioSource::updateCoordinate(
        std::string &head_frame_id,
        tf2_ros::Buffer &tf_buffer,
        ALCcontext *context )
{
    try {
        this->source_transform_ =
            tf_buffer.lookupTransform(
                    head_frame_id.c_str(),
                    this->source_frame_id_.c_str(),
                    ros::Time(0)
                    );
    } catch (const tf2::LookupException& e) {
        ROS_ERROR( "%s", e.what() );
        return;
    }

    alcSuspendContext( context );
    {
        ALfloat pos_source[3];
        pos_source[0] = this->source_transform_.transform.translation.x;
        pos_source[1] = this->source_transform_.transform.translation.y;
        pos_source[2] = this->source_transform_.transform.translation.z;
        alSourcefv( this->al_source_id_, AL_POSITION, pos_source );
    }
    alcProcessContext( context );
}

void SpatialAudioSource::dequeALBuffers()
{
    ALsizei n, m;
    this->mtx_.lock();
    alGetSourcei( this->al_source_id_, AL_BUFFERS_PROCESSED, &n ); // get a number of buffers processed.
    ALuint* buffers = new ALuint[n]; // These buffers should be released. Maybe shared_ptr should be used.
    alSourceUnqueueBuffers( this->al_source_id_, n, buffers ); // unqueue the buffers
    this->mtx_.unlock();
    alDeleteBuffers( n, buffers );
    delete[] buffers;
    alGetSourcei( this->al_source_id_, AL_BUFFERS_QUEUED, &m );

    ROS_INFO( "id:%d, %d buffers processed, %d buffers remains.", this->id_, n, m );
}

ALint SpatialAudioSource::getSourceState()
{
    ALint source_state;
    alGetSourcei( this->al_source_id_, AL_SOURCE_STATE, &source_state );
    return source_state;
}

void SpatialAudioSource::startSourcePlay()
{
    alSourcePlay( this->al_source_id_ );
}

int SpatialAudioSource::getAudioSourceID()
{
    return this->id_;
}

void SpatialAudioSource::callbackAudioStream( const boost::shared_ptr<audio_stream_msgs::AudioData const>& ptr_msg )
{
    this->dequeALBuffers();

    // enqueue a new buffer with recieved data
    ALuint buffer_id;
    ALsizei buffer_size = ptr_msg->data.size();
    genBufferFromPCM(
            buffer_id,
            (ALvoid*)ptr_msg->data.data(),
            buffer_size,
            this->stream_sampling_rate_,
            this->stream_format_ );
    this->mtx_.lock();
    alSourceQueueBuffers( this->al_source_id_, 1, &buffer_id );
    this->mtx_.unlock();

    ROS_DEBUG( "New data containes %d bytes.", buffer_size );
}


}
