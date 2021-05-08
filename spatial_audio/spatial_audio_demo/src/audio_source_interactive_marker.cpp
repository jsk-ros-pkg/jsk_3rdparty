// Standard C++
#include <iostream>
#include <mutex>
// Boost
#include <boost/shared_ptr.hpp>
// ROS
#include <geometry_msgs/TransformStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>

/**
 * global values
 */
std::string fixed_frame_id;
std::string source_frame_id;
geometry_msgs::TransformStamped transformStamped;
std::mutex mtx;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    mtx.lock();
    transformStamped.header.frame_id = fixed_frame_id;
    transformStamped.child_frame_id = source_frame_id;
    transformStamped.transform.translation.x = feedback->pose.position.x;
    transformStamped.transform.translation.y = feedback->pose.position.y;
    transformStamped.transform.translation.z = feedback->pose.position.z;
    transformStamped.transform.rotation = feedback->pose.orientation;
    mtx.unlock();
}

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "audio_source_interactive_marker" );

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double base_scale, marker_scale;
    double position_x, position_y, position_z;
    double color_r, color_g, color_b, color_a;
    std::string name, description;

    nh_private.param<std::string>("description",     description,     "Source Position");
    nh_private.param<std::string>("name",            name,            "audio_source");
    nh_private.param<std::string>("fixed_frame_id",  fixed_frame_id,  "map" );
    nh_private.param<std::string>("source_frame_id", source_frame_id, "source" );
    nh_private.param<double>     ("base_scale",      base_scale,      1 );
    nh_private.param<double>     ("marker_scale",    marker_scale,    1 );
    nh_private.param<double>     ("position_x",      position_x,      1.0 );
    nh_private.param<double>     ("position_y",      position_y,      0.0 );
    nh_private.param<double>     ("position_z",      position_z,      0.0 );
    nh_private.param<double>     ("color_r",         color_r,         0.5 );
    nh_private.param<double>     ("color_g",         color_g,         0.5 );
    nh_private.param<double>     ("color_b",         color_b,         0.5 );
    nh_private.param<double>     ("color_a",         color_a,         1.0 );

    interactive_markers::InteractiveMarkerServer server("interactive_marker");

    visualization_msgs::InteractiveMarker int_marker;

    int_marker.header.frame_id = fixed_frame_id;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = name;
    int_marker.description = "Source Position";
    int_marker.scale = base_scale;
    int_marker.pose.position.x = position_x;
    int_marker.pose.position.y = position_y;
    int_marker.pose.position.z = position_z;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = 0.0;
    int_marker.pose.orientation.w = 1.0;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = marker_scale;
    marker.scale.y = marker_scale;
    marker.scale.z = marker_scale;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = color_a;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( marker );
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    int_marker.controls.push_back( control );

    static tf2_ros::TransformBroadcaster br;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = fixed_frame_id;
    transformStamped.child_frame_id = source_frame_id;
    transformStamped.transform.translation.x = int_marker.pose.position.x;
    transformStamped.transform.translation.y = int_marker.pose.position.y;
    transformStamped.transform.translation.z = int_marker.pose.position.z;
    transformStamped.transform.rotation = int_marker.pose.orientation;

    server.insert( int_marker, &processFeedback );
    server.applyChanges();

    boost::shared_ptr<ros::AsyncSpinner> ptr_spinner( new ros::AsyncSpinner(1) );
    ptr_spinner->start();

    ros::Rate r(10);
    while (ros::ok()) {
        mtx.lock();
        transformStamped.header.stamp = ros::Time::now();
        br.sendTransform( transformStamped );
        mtx.unlock();
        r.sleep();
        ROS_INFO("broadcasted a transform");
    }
}
