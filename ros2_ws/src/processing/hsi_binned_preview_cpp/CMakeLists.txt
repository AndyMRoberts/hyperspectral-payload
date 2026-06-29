cmake_minimum_required(VERSION 3.14)
project(hsi_binned_preview_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPreviewType.srv"
)

add_executable(hsi_binned_preview_node src/hsi_binned_preview_node.cpp)
ament_target_dependencies(hsi_binned_preview_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(hsi_binned_preview_node ${OpenCV_LIBS})
target_include_directories(hsi_binned_preview_node PRIVATE ${OpenCV_INCLUDE_DIRS})
rosidl_target_interfaces(hsi_binned_preview_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS hsi_binned_preview_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
