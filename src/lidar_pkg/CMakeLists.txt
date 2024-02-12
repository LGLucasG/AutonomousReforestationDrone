cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include C++ headers
include_directories(include)

# Declare a C++ executable
add_executable(lidar_node src/lidar_node.cpp src/lidarlite_v3.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(lidar_node rclcpp std_msgs)

# Install targets
install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# ... (other configurations)

ament_package()
