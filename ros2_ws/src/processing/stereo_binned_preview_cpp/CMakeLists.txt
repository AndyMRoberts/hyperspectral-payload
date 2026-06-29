cmake_minimum_required(VERSION 3.14)
project(stereo_binned_preview_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stereo_binned_preview_node src/stereo_binned_preview_node.cpp)
ament_target_dependencies(stereo_binned_preview_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(stereo_binned_preview_node ${OpenCV_LIBS})
target_include_directories(stereo_binned_preview_node PRIVATE ${OpenCV_INCLUDE_DIRS})

install(TARGETS stereo_binned_preview_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
