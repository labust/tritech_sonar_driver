#ifndef MICRON_SONAR_NODE_HPP
#define MICRON_SONAR_NODE_HPP

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud.hpp>
#include "drivers_sonar_tritech/SeaNetMicron.hpp"
#include "drivers_sonar_tritech/UdpDriver.hpp"
#include "rclcpp/rclcpp.hpp"

class SonarNode : public rclcpp::Node {
  public:
   SonarNode();

  private:
   void timer_callback();
   void publish_point_cloud();
   void publish_sonar_heading();
   void declare_parameters();

   bool debug_;
   double max_distance_;
   double min_distance_;
   double gain_;
   double frequency_out_;         // Hz
   double speed_of_propagation_;  // m/s default like water

   std::shared_ptr<sea_net::Micron> micron_driver_;
   std::shared_ptr<UDPDriver> udp_driver_;

   rclcpp::TimerBase::SharedPtr timer_;

   int baudrate_;
   int udpPort_;
   std::string udpServer_;
   std::string port_;
   base::samples::SonarBeam sonar_beam_;
   rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
       point_cloud_publisher_;
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
       pose_publisher_;
   sea_net::MicronConfig config_;

   double left_limit_;

   double right_limit_;

   double resolution_;

   double angular_resolution_;

   bool low_resolution_;

   bool continous_;

   bool invert_;

   bool stare_left_limit_;

   int timeout_receive_data_;  // ms
};

#endif  // MICRON_SONAR_NODE_HPP
