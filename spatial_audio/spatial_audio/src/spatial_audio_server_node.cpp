// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// USER
#include <spatial_audio/spatial_audio_server.h>

int main( int argc, char** argv )
{
    //
    ros::init( argc, argv, "play_sin_spatially" );
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //
    double max_duration;
    nh_private.param<double>("max_duration", max_duration, 5.0);

    //
    tf2_ros::Buffer            tf_buffer(ros::Duration(max_duration*2));
    tf2_ros::TransformListener tf_listener(tf_buffer);

    //
    spatial_audio::SpatialAudioServer server( nh, nh_private, tf_buffer );
    server.spin( 10 );
}
