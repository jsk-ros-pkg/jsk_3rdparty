// doc/html/boost_asio/example/icmp/

#include "ping.cpp"
// ROS1 (roscpp) when <ros/ros.h> is available, which defines
// ROS_VERSION_MAJOR to 1, else ROS2 (rclcpp)
#if defined(__has_include)
#if __has_include(<ros/ros.h>)
#include <ros/ros.h>
#endif
#else
#include <ros/ros.h>
#endif
#if ROS_VERSION_MAJOR == 1
#include <std_msgs/Float64.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <unistd.h>
#endif

int main(int argc, char* argv[])
{
  try
      {
#if ROS_VERSION_MAJOR == 1
          ros::init(argc, argv, "ping");
#else
          rclcpp::init(argc, argv);
          // without the --ros-args, as ros::init() removes the remappings
          std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
          std::vector<char*> args_c;
          for (std::string& arg : args) {
            args_c.push_back(&arg[0]);
          }
          argc = args_c.size();
          argv = args_c.data();
#endif
          if (argc != 2)
              {
                  std::cerr << "Usage: " << argv[0] << " <host>" << std::endl;
#if !defined(BOOST_WINDOWS)
                  std::cerr << "(You may need to set permission to run this program as root.)\n$ chown root.root rosping; ls -al rosping; chmod 4755 rosping\n" << std::endl;
#endif
                  return 1;
              }

          boost::asio::io_service io_service;
          pinger p(io_service, argv[1]);

          

#if ROS_VERSION_MAJOR == 1
          ros::NodeHandle n;
          ros::NodeHandle pnh("~");
          double rate;
          if (!pnh.getParam("rate", rate)) {
            rate = 10.0;         // 0.1Hz
          }
          ros::Publisher pub = n.advertise<std_msgs::Float64>("ping/delay", 10);

          std_msgs::Float64 msg;

          ros::Time last_time = ros::Time::now();

          while (ros::ok()) {
              ros::Time now = ros::Time::now();
              io_service.run_one();              
              if ( now - last_time > ros::Duration(rate) ) {

                  msg.data = p.delay;
                  pub.publish(msg);
                  ros::spinOnce();

                  last_time = now;
              }
          }
#else
          // the raw socket is open: drop the root privileges of the setuid
          // install before creating the node, else the shared memory files
          // of DDS (Fast DDS's /dev/shm/fastrtps_*) are owned by root, and
          // the nodes of the user do not receive ping/delay through them
          if (setuid(getuid()) != 0) {
              throw std::runtime_error("failed to drop the root privileges");
          }
          auto n = rclcpp::Node::make_shared("ping");
          double rate = n->declare_parameter<double>("rate", 10.0);   // 0.1Hz
          auto pub = n->create_publisher<std_msgs::msg::Float64>("ping/delay", 10);

          std_msgs::msg::Float64 msg;

          rclcpp::Time last_time = n->now();

          while (rclcpp::ok()) {
              rclcpp::Time now = n->now();
              io_service.run_one();
              if ( now - last_time > rclcpp::Duration::from_seconds(rate) ) {

                  msg.data = p.delay;
                  pub->publish(msg);
                  rclcpp::spin_some(n);

                  last_time = now;
              }
          }
          rclcpp::shutdown();
#endif

      }
  catch (std::exception& e)
      {
          std::cerr << "Exception: " << e.what() << std::endl;
      }
}
