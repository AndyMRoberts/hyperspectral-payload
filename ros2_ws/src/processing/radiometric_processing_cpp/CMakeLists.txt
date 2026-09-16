cmake_minimum_required(VERSION 3.14)
project(radiometric_processing_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

add_library(fast_radiometric_processing
  src/fast_radiometric_processing.cpp
)
target_include_directories(fast_radiometric_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIRS}
)
target_link_libraries(fast_radiometric_processing
  ${OpenCV_LIBS}
  ${TINYXML2_LIBRARIES}
)

add_executable(radiometric_processing_node
  src/radiometric_processing_node.cpp
)
target_include_directories(radiometric_processing_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(radiometric_processing_node
  rclcpp
  sensor_msgs
  std_msgs
  custom_msgs
)
target_link_libraries(radiometric_processing_node
  fast_radiometric_processing
  ${OpenCV_LIBS}
)

install(TARGETS radiometric_processing_node fast_radiometric_processing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
