cmake_minimum_required(VERSION 3.8)
project(hsi_camera_driver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CameraCommands.srv"
)

add_executable(hsi_camera_node src/hsi_camera_node.cpp)
target_compile_features(hsi_camera_node PUBLIC cxx_std_17)

rosidl_get_typesupport_target(
  cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hsi_camera_node "${cpp_typesupport_target}")

target_include_directories(hsi_camera_node PRIVATE
  /opt/imec/hsi-mosaic/include
  /opt/XIMEA/include
)

target_link_directories(hsi_camera_node PRIVATE
  /opt/imec/hsi-mosaic/bin
)

target_link_libraries(hsi_camera_node
  hsi_api
  hsi_camera_api
  m3api
)

set_target_properties(hsi_camera_node PROPERTIES
  BUILD_RPATH "/opt/imec/hsi-mosaic/bin"
  INSTALL_RPATH "/opt/imec/hsi-mosaic/bin"
)

ament_target_dependencies(hsi_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS hsi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
