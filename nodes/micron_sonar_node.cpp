#include "ros_nodes/micron_sonar_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"

SonarNode::SonarNode() : Node("sonar_node") {

   declare_parameters();

   micron_driver_ = std::make_shared<sea_net::Micron>(debug_);
   udp_driver_ = std::make_shared<UDPDriver>(debug_);


   if (should_turn_on_motor_ && stare_left_limit_){
      turn_on_motor();
   };

   if (!udpServer_.empty()) {
      if (!udp_driver_->init(std::string(udpServer_), udpPort_)) {
         RCLCPP_ERROR(this->get_logger(), "Could not open UDP server");
      }
   }

   micron_driver_->openSerial(port_, baudrate_);
   base::samples::RigidBodyState rbs;  // FIXME: why is it here?
   configurin_w_staring();
   int period_ms = static_cast<int>(1000.0 / frequency_out_);

   point_cloud_publisher_ =
       this->create_publisher<sensor_msgs::msg::PointCloud>(
           "micron_sonar/point_cloud", 10);
   pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
       "micron_sonar/heading", 10);

   if (stare_left_limit_) {
      distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
          "micron_sonar/distance", 10);
   }


   previous_time_sonar_timer_ = std::chrono::high_resolution_clock::now();
   timer_ =
       this->create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&SonarNode::timer_callback, this));
}

void SonarNode::configurin_w_staring(){
   RCLCPP_INFO(this->get_logger(), "configuring with staring");
   micron_driver_->configure(config_, 10000, stare_left_limit_);
   RCLCPP_INFO(this->get_logger(), "requesting data");
   micron_driver_->requestData();
}

void SonarNode::turn_on_motor(){
   std::cout<<std::endl;
   std::cout<<std::endl;
   std::cout<<std::endl;
   std::cout<<std::endl;
    RCLCPP_INFO(this->get_logger(), "turning motor on");
   try{
         std::unique_ptr<sea_net::Micron> micron_driver_mock = std::make_unique<sea_net::Micron>(debug_);
         std::unique_ptr<UDPDriver> udp_driver_mock = std::make_unique<UDPDriver>(debug_);

         if (!udpServer_.empty()) {
            if (!udp_driver_mock->init(std::string(udpServer_), udpPort_)) {
               RCLCPP_ERROR(this->get_logger(), "Could not open MOCK UDP server");
            }
         }
         micron_driver_mock->openSerial(port_, baudrate_);
         base::samples::RigidBodyState rbs_mock;
         // Use sector limits even during motor startup
         micron_driver_mock->configure(config_, 10000, stare_left_limit_);
         micron_driver_mock->requestData();
         mock_timer_callback(micron_driver_mock, udp_driver_mock);
         RCLCPP_INFO(this->get_logger(), "tried to config without staring");

      } catch (const std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Exception during initial configuration: %s", e.what());
      }
}

void SonarNode::declare_parameters() {

   this->declare_parameter<bool>("should_turn_on_motor", true);
   this->get_parameter("should_turn_on_motor", should_turn_on_motor_);

   this->declare_parameter<bool>("debug", false);
   this->get_parameter("debug", debug_);

   this->declare_parameter<int>("baudrate", 115200);
   this->get_parameter("baudrate", baudrate_);

   this->declare_parameter<int>("udpPort", 4000);
   this->get_parameter("udpPort", udpPort_);

   this->declare_parameter<std::string>("udpServer", "");
   this->get_parameter("udpServer", udpServer_);

   this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
   this->get_parameter("port", port_);

   this->declare_parameter<double>("max_distance", 30);
   this->get_parameter("max_distance", max_distance_);

   this->declare_parameter<double>("min_distance", 0.01);
   this->get_parameter("min_distance", min_distance_);

   this->declare_parameter<double>("frequency_out", 5);
   this->get_parameter("frequency_out", frequency_out_);

   this->declare_parameter<double>("gain", 0.4);
   this->get_parameter("gain", gain_);

   this->declare_parameter<double>("speed_of_propagation", 1482.0);
   this->get_parameter("speed_of_propagation", speed_of_propagation_);

   this->declare_parameter<double>("left_limit", 180.0);
   this->get_parameter("left_limit", left_limit_);

   this->declare_parameter<double>("right_limit", -180.0);
   this->get_parameter("right_limit", right_limit_);

   this->declare_parameter<double>("angular_resolution", 5.0);
   this->get_parameter("angular_resolution", angular_resolution_);

   this->declare_parameter<double>("resolution", 0.1);
   this->get_parameter("resolution", resolution_);

   this->declare_parameter<bool>("low_resolution", false);
   this->get_parameter("low_resolution", low_resolution_);

   this->declare_parameter<bool>("continous", true);
   this->get_parameter("continous", continous_);

   this->declare_parameter<bool>("invert", false);
   this->get_parameter("invert", invert_);

   this->declare_parameter<bool>("stare_left_limit", false);
   this->get_parameter("stare_left_limit", stare_left_limit_);

   this->declare_parameter<float>("intensity_threshold", 75.0);
   this->get_parameter("intensity_threshold", intensity_threshold_);

   this->declare_parameter<float>("min_dist_considered", 0.55);
   this->get_parameter("min_dist_considered", min_dist_considered_);

   this->declare_parameter<float>("peak_proportion", 2.5);
   this->get_parameter("peak_proportion", peak_proportion_);

   this->declare_parameter<int>("noise_counter_threshold", 10);
   this->get_parameter("noise_counter_threshold", noise_counter_threshold_);

   this->declare_parameter<int>("timeout_receive_data", 790);
   this->get_parameter("timeout_receive_data", timeout_receive_data_);
   
   config_.max_distance = max_distance_;
   config_.min_distance = min_distance_;
   config_.gain = gain_;
   config_.speed_of_sound = speed_of_propagation_;
   config_.left_limit = base::Angle::fromDeg(left_limit_);
   config_.right_limit = base::Angle::fromDeg(right_limit_);
   config_.angular_resolution = base::Angle::fromDeg(angular_resolution_);
   config_.resolution = resolution_;
   config_.low_resolution = low_resolution_;
   config_.continous = continous_;
   config_.invert = invert_;

   timer_counter_ = 0;
   timer_threshold_ = 5;

   last_distance_ = 0.0;
   is_peak_detected_ = false;
}

void SonarNode::mock_timer_callback(std::unique_ptr<sea_net::Micron>& micron_driver, std::unique_ptr<UDPDriver>& udp_driver){
    auto formatted_now = micron_driver->formattedNow();
   try{
      if (debug_) {
         RCLCPP_INFO(this->get_logger(), "MOCK Timer callback executed at: %s",formatted_now.c_str());
      }

      micron_driver->receiveData(timeout_receive_data_);
      if (debug_) {
         RCLCPP_INFO(this->get_logger(), "MOCK receiveData at: %s",formatted_now.c_str());
      }

      micron_driver->requestData();
      if (debug_) {
         RCLCPP_INFO(this->get_logger(), "MOCK requestData at: %s",formatted_now.c_str());
      }


      base::samples::Sonar sonar;
      if (debug_) {
         RCLCPP_INFO(this->get_logger(), "MOCK sonar at: %s",formatted_now.c_str());
      }
      
      micron_driver->decodeSonar(sonar);
      if (debug_) {
         RCLCPP_INFO(this->get_logger(), "MOCK decodeSonar at: %s",formatted_now.c_str());
      }
      
      sonar_beam_ = sonar.toSonarBeam();
      if (debug_) {
         RCLCPP_INFO(this->get_logger(), "MOCK toSonarBeam at: %s",formatted_now.c_str());
      }

      if (!udpServer_.empty()) {
         udp_driver->sendSonarBeam(sonar_beam_);
      }
      RCLCPP_INFO(this->get_logger(), "MOCK: %s",formatted_now.c_str());
   } catch (const std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Exception during mock_cb: %s", e.what());
      }
   
}

void SonarNode::timer_callback() {
   auto formatted_now = micron_driver_->formattedNow();
   auto current_time = std::chrono::high_resolution_clock::now();


   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "Timer callback executed at: %s",formatted_now.c_str());
   }
   if(should_turn_on_motor_){
      if (previous_time_sonar_timer_ != std::chrono::high_resolution_clock::time_point()) {
         auto time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - previous_time_sonar_timer_).count();
         
         if (time_diff >= 0.2) {
            auto current_time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count();
            auto previous_time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(previous_time_sonar_timer_.time_since_epoch()).count();
            RCLCPP_INFO(this->get_logger(), "WITHIN - Current time (ms since epoch): %ld, Previous time (ms since epoch): %ld",
                  current_time_duration, previous_time_duration);
               RCLCPP_INFO(this->get_logger(), "Time difference: %.6f seconds", time_diff);
               if(timer_counter_<timer_threshold_){
                  timer_counter_+=1;
               } else {
                  turn_on_motor();
                  // configurin_w_staring();
                  timer_counter_ = 0;
               }
         } else {
            timer_counter_ = 0;
         }
      }
   }
   
   micron_driver_->receiveData(timeout_receive_data_);
   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "receiveData at: %s",formatted_now.c_str());
   }

   micron_driver_->requestData();
   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "requestData at: %s",formatted_now.c_str());
   }


   base::samples::Sonar sonar;
   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "sonar at: %s",formatted_now.c_str());
   }
   
   micron_driver_->decodeSonar(sonar);
   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "decodeSonar at: %s",formatted_now.c_str());
   }
   
   sonar_beam_ = sonar.toSonarBeam();
   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "toSonarBeam at: %s",formatted_now.c_str());
   }
   
   double bearing_rad = sonar_beam_.bearing.rad;
   double left_limit_rad = config_.left_limit.rad;
   double right_limit_rad = config_.right_limit.rad;
   
   // Normalize bearing to [-PI, PI] range
   while (bearing_rad > M_PI) bearing_rad -= 2.0 * M_PI;
   while (bearing_rad < -M_PI) bearing_rad += 2.0 * M_PI;
   
   // Check if bearing is within sector
   bool in_sector = false;
   if (left_limit_rad >= right_limit_rad) {
      in_sector = (bearing_rad <= left_limit_rad && bearing_rad >= right_limit_rad);
   } else {
      in_sector = (bearing_rad <= left_limit_rad || bearing_rad >= right_limit_rad);
   }
   
   if (!in_sector && !stare_left_limit_) {
      previous_time_sonar_timer_ = current_time;
      return;
   }
   
   publish_point_cloud();
   publish_sonar_heading();
   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "publishing at: %s",formatted_now.c_str());
   }
   
   
   if (!udpServer_.empty()) {
      udp_driver_->sendSonarBeam(sonar_beam_);
   }

   previous_time_sonar_timer_ = current_time;
}

void SonarNode::publish_point_cloud() {
   sensor_msgs::msg::PointCloud point_cloud_msg;
   point_cloud_msg.header.stamp = this->now();
   point_cloud_msg.header.frame_id = "sonar_frame";

   float r_step = sonar_beam_.getSpatialResolution();

   sensor_msgs::msg::ChannelFloat32 channel;
   channel.name = "intensity";
   bool is_distance_found = false;
   float distance_first_peak = 0;

   if (debug_) {
      std::cout << "beam(size: " << sonar_beam_.beam.size() << "): ";
   }

   for (size_t i = 0; i < sonar_beam_.beam.size(); ++i) {
      float range = r_step * (i + 1);
      // if (range < 0.55 || range < config_.min_distance || range > config_.max_distance) {
      //    continue;
      // }

      if (range < config_.min_distance || range > config_.max_distance) {
         continue;
      }
      // float half_bw_hor = sonar_beam_.beamwidth_horizontal / 2;
      float x_unit = std::cos(sonar_beam_.bearing.rad) * range;
      float y_unit = std::sin(sonar_beam_.bearing.rad) * range;

      geometry_msgs::msg::Point32 point;
      point.x = x_unit;
      point.y = y_unit;
      point.z = 0.0;

      point_cloud_msg.points.push_back(point);
      float sonar_beam_float = static_cast<float>(sonar_beam_.beam[i]);
      channel.values.push_back(sonar_beam_float);

      int data_value = static_cast<int>(sonar_beam_.beam[i]);
      if (debug_) {
         std::cout << std::setw(3) << std::setfill('0') << data_value << ", ";
      }

      float distance_point = sqrt(x_unit * x_unit + y_unit * y_unit);
      if (stare_left_limit_) {
         if ((!is_distance_found)  
             && (i > 0)
             && (distance_point > min_dist_considered_)
             && (sonar_beam_float >= peak_proportion_ * sonar_beam_.beam[i - 1])
             && ((sonar_beam_float >= intensity_threshold_)
             || (sonar_beam_.beam[i + 1] >= intensity_threshold_*0.7) )
            //  && (sonar_beam_.beam[i + 2] > 0.0)
            //  && (sonar_beam_.beam[i + 3] >= intensity_threshold_*0.5)
             ) {
            distance_first_peak = sqrt(x_unit * x_unit + y_unit * y_unit);
            is_distance_found = true;
         }
      }
   }

   // Publish distance if stare_left_limit
   if (stare_left_limit_) {
      std_msgs::msg::Float64 dist_msg;
      if(is_distance_found) {  
         dist_msg.data = distance_first_peak;
         noise_counter_=0;
         if (debug_) {
            std::cout << "distance from observed point: " << distance_first_peak << std::endl;
         }
      } else {
         if(is_peak_detected_){
            if(last_distance_ <= ((resolution_*1.5)+ min_dist_considered_)){
               // too close
               std::cout << "too close("<<distance_first_peak  <<") - last_distance_: " << last_distance_ << " - (setting dist_msg.data = -1)"<< std::endl;
               dist_msg.data = -1;
               noise_counter_=0;
            } else if (last_distance_ >= (max_distance_ - (resolution_*1.5))) {
               // too far
              std::cout << "too far("<<distance_first_peak  <<") - last_distance_: " << last_distance_ << " - (setting dist_msg.data = -2)"<< std::endl;
               dist_msg.data = -2;    
               noise_counter_=0;     
            } else {
              if(noise_counter_ >= noise_counter_threshold_){
               // probably noise, keeping last distance
              std::cout << "WARNING - too many noisy measurements("<<distance_first_peak  <<"): " << noise_counter_ << ", do something! "<< std::endl;
               // FIXME: implement logic for too many noisy measurements
               distance_first_peak = last_distance_;
               dist_msg.data = distance_first_peak; 
              } else {
               // probably noise, keeping last distance
              std::cout << "probably noise ("<< distance_first_peak <<"), keeping last distance: " << last_distance_<< std::endl;
               distance_first_peak = last_distance_;
               dist_msg.data = distance_first_peak; 
              }
              
               noise_counter_+=1;
            }
         } else {
            // peak never detected
            std::cout << "peak never detected - (setting dist_msg.data = -3)"<< std::endl;
            dist_msg.data = -3;
            noise_counter_=0;
         }
         
      }
      distance_publisher_->publish(dist_msg);
   }
   if (debug_) {
      std::cout << std::endl;
   }
   point_cloud_msg.channels.push_back(channel);
   point_cloud_publisher_->publish(point_cloud_msg);
   last_distance_ = distance_first_peak;
   if (last_distance_ > 0){
      is_peak_detected_ = true;
   }
}

void SonarNode::publish_sonar_heading() {
   geometry_msgs::msg::PoseStamped pose_msg;
   pose_msg.header.stamp = this->now();
   pose_msg.header.frame_id = "sonar_frame";

   tf2::Quaternion q;
   q.setRPY(0, 0, sonar_beam_.bearing.rad);

   pose_msg.pose.orientation = tf2::toMsg(q);
   pose_msg.pose.position.x = 0.0;
   pose_msg.pose.position.y = 0.0;
   pose_msg.pose.position.z = 0.0;

   pose_publisher_->publish(pose_msg);
}