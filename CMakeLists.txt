cmake_minimum_required(VERSION 3.5)
project(drivers_sonar_tritech)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Define the library
add_library(sonar_tritech
  drivers/Angle.cpp
  drivers/Driver.cpp
  drivers/Exceptions.cpp
  drivers/IOListener.cpp
  drivers/IOStream.cpp
  drivers/LaserScan.cpp
  drivers/Pose.cpp
  drivers/RigidBodyState.cpp
  drivers/SeaNet.cpp
  drivers/SeaNetEchosounder.cpp
  drivers/SeaNetMicron.cpp
  drivers/SeaNetProfiling.cpp
  drivers/SeaNetTypesInternal.cpp
  drivers/SerialConfiguration.cpp
  drivers/Sonar.cpp
  drivers/SonarBeam.cpp
  drivers/SonarScan.cpp
  drivers/TestStream.cpp
  drivers/Time.cpp
  drivers/Timeout.cpp
  drivers/TimestampEstimator.cpp
  drivers/UdpDriver.cpp
  drivers/URI.cpp
)

# Specify include files for the library
target_include_directories(sonar_tritech PUBLIC
  include
)

# Link dependencies for the library
ament_target_dependencies(sonar_tritech
  rclcpp
  std_msgs
  Eigen3

)

target_link_libraries(sonar_tritech
  ${Boost_LIBRARIES}
)

add_library(tritech_nodes
  nodes/micron_sonar_node.cpp
)

# Specify include files for the library
target_include_directories(tritech_nodes PUBLIC
  include
  sonar_tritech
)

# Link dependencies for the library
ament_target_dependencies(tritech_nodes
  rclcpp
  std_msgs
  Eigen3
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(tritech_nodes
  sonar_tritech
  ${Boost_LIBRARIES}
)


add_executable(echosounder_scan src/EchosounderScan.cpp)
add_executable(micron_sonar_node src/micron_sonar_node_main.cpp)

ament_target_dependencies(micron_sonar_node
  rclcpp
  std_msgs
  Eigen3
)

target_link_libraries(micron_sonar_node
  tritech_nodes
  ${Boost_LIBRARIES}
)

ament_target_dependencies(echosounder_scan
  rclcpp
  std_msgs
  Eigen3
)

target_link_libraries(echosounder_scan
  sonar_tritech
  ${Boost_LIBRARIES}
)

# Install targets
install(TARGETS 
  echosounder_scan
  micron_sonar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    # config
    # models
    # maps
    rviz
    launch
    # params
  #  urdf
   # meshes
  #  worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
