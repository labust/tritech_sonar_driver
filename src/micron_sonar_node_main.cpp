#include "rclcpp/rclcpp.hpp"
#include "ros_nodes/micron_sonar_node.hpp"

int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<SonarNode>();
   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();
   rclcpp::shutdown();
   return 0;
}
