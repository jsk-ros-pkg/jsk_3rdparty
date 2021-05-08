#ifndef SPATIAL_AUDIO_SPATIAL_AUDIO_SERVER_H__
#define SPATIAL_AUDIO_SPATIAL_AUDIO_SERVER_H__

// Standaerd C++ Library
#include <iostream>
#include <list>
#include <mutex>
#include <string>
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
// USER
#include <spatial_audio/spatial_audio_source.h>

namespace spatial_audio {

    class SpatialAudioServer
    {
        public:
            /**
             * Constructor
             * @param[in] nh ros node handler
             * @param[in] nh_private ros node handler with private namespace
             * @param[in] tf_buffer tf2_ros buffer
             */
            SpatialAudioServer( ros::NodeHandle& nh,
                                ros::NodeHandle& nh_private,
                                tf2_ros::Buffer& tf_buffer );
            ~SpatialAudioServer();

            /**
             * spin function
             * @param[in] spin_rate spin rate for main loop [hz]
             */
            void spin( int spin_rate );

        private:
            /**
             * ROS serivice handler
             */
            bool handlerPlayService( 
                    spatial_audio_msgs::PlaySpatialAudio::Request &req,
                    spatial_audio_msgs::PlaySpatialAudio::Response &res );
            /**
             * add an audio source
             * @param[in] req a request of an audio source to add
             */
            bool addSource( spatial_audio_msgs::PlaySpatialAudio::Request &req );
            /**
             * delete an audio source
             * @param[in] id id number of an audio source to delete
             */
            bool delSource( int id );
            /**
             * check if there is an audio source object with a specified id
             * @param[in] id id number of an audio source to check
             */
            std::list<SpatialAudioSource>::iterator findSource( int id );
            /**
             * update transformation values from head frame to each source frame.
             */
            void updateCoordinates();


            /**
             * OpenAL resources
             */
            ALCcontext* context_;
            ALCdevice* device_;

            /**
             * ROS related resources
             */
            ros::NodeHandle& nh_;
            ros::NodeHandle& nh_private_;
            tf2_ros::Buffer& tf_buffer_;
            ros::ServiceServer srv_;
            boost::shared_ptr<ros::AsyncSpinner> ptr_spinner_;
            std::string head_frame_id_;

            /**
             * audio source management related
             */
            std::list<SpatialAudioSource> list_audio_source_;
            std::mutex mtx_audio_source_;
    };
}

#endif
