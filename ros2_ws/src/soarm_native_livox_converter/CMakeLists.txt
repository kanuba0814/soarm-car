cmake_minimum_required(VERSION 3.8)
project(soarm_native_livox_converter)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_core src/converter.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_core livox_ros_driver2 sensor_msgs)

add_executable(native_livox_converter src/native_livox_converter.cpp)
target_link_libraries(native_livox_converter ${PROJECT_NAME}_core)
ament_target_dependencies(native_livox_converter livox_ros_driver2 rclcpp sensor_msgs)

install(TARGETS ${PROJECT_NAME}_core native_livox_converter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(converter_test test/converter_test.cpp)
  target_link_libraries(converter_test ${PROJECT_NAME}_core)
  ament_target_dependencies(converter_test livox_ros_driver2 rclcpp sensor_msgs)
  target_compile_definitions(converter_test PRIVATE RAW_CLOUD_CDR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/raw-cloud.cdr")
endif()

ament_package()
