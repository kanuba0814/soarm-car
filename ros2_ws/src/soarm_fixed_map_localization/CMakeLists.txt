cmake_minimum_required(VERSION 3.16)
project(soarm_fixed_map_localization LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(soarm_localization REQUIRED)

add_library(alignment_state src/alignment_state.cpp)
target_compile_features(alignment_state PUBLIC cxx_std_17)
target_include_directories(
    alignment_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    alignment_state PUBLIC Eigen3::Eigen soarm_localization::soarm_localization
)

add_executable(fixed_map_shadow_node src/fixed_map_shadow_node.cpp)
target_compile_features(fixed_map_shadow_node PRIVATE cxx_std_17)
target_link_libraries(
    fixed_map_shadow_node alignment_state Eigen3::Eigen ${PCL_LIBRARIES}
    soarm_localization::soarm_localization
)
ament_target_dependencies(
    fixed_map_shadow_node rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs
    pcl_conversions
)

install(
    TARGETS alignment_state fixed_map_shadow_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 soarm_localization)

if(BUILD_TESTING)
    enable_testing()
    add_executable(alignment_state_test test/alignment_state_test.cpp)
    target_link_libraries(alignment_state_test PRIVATE alignment_state)
    target_compile_options(alignment_state_test PRIVATE -UNDEBUG)
    add_test(NAME alignment_state_test COMMAND alignment_state_test)
endif()

ament_package()
