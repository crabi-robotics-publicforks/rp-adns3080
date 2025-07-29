cmake_minimum_required(VERSION 3.5)
project(adns3080_ros2)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(adns3080_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(adns3080_publisher
  src/adns3080_pub.cpp
  src/camera.cpp
)

ament_target_dependencies(adns3080_publisher
  rclcpp
  adns3080_interfaces
  geometry_msgs
)

target_link_libraries(adns3080_publisher bcm2835)

install(TARGETS
  adns3080_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
