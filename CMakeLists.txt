cmake_minimum_required(VERSION 3.8)
project(dwe_ros2_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# add exe
add_executable(dwe_ros2_parser src/main.cc src/dwe_ros2_parser.cc)
add_executable(image_sub src/image_subscriber.cc)

# include dirs
include_directories(include)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# add depend
ament_target_dependencies(dwe_ros2_parser
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_target_dependencies(image_sub
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# install targets
install(TARGETS
  dwe_ros2_parser
  image_sub
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()