#ifndef MICRON_SONAR_NODE_HPP
#define MICRON_SONAR_NODE_HPP

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/float64.hpp>
#include "drivers_sonar_tritech/SeaNetMicron.hpp"
#include "drivers_sonar_tritech/UdpDriver.hpp"
#include "rclcpp/rclcpp.hpp"

class SonarNode : public rclcpp::Node {
  public:
   SonarNode();

  private:
   void timer_callback();
   void mock_timer_callback(std::unique_ptr<sea_net::Micron>& micron_driver, std::unique_ptr<UDPDriver>& udp_driver);
   void turn_on_motor();
   void configurin_w_staring();
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
   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;
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

   float intensity_threshold_;  // value indicating the threshold for a solid
                              // object detected
   float min_dist_considered_; // min distance considered for threshold - m

   float peak_proportion_; // proportion of first peak compared to previous number

   float last_distance_; // last peak detected

   float sonar_step_; // m

   bool is_peak_detected_;

   bool should_turn_on_motor_; // turn on motor even if you are in stare_mode

   int noise_counter_; // to keep track of how many noisy detections

   int noise_counter_threshold_; // how many consecutives noise measurements are allowed

   int timer_counter_;
   
   int timer_threshold_;

   std::chrono::high_resolution_clock::time_point previous_time_sonar_timer_;
};

#endif  // MICRON_SONAR_NODE_HPP
