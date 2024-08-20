cmake_minimum_required(VERSION 3.8)
project(prometheus_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  topic_monitor_parameters
  parameters/topic_monitor.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_monitor_component.cpp
  src/message_info_subscription.cpp
)
target_link_libraries(${PROJECT_NAME} topic_monitor_parameters)
rclcpp_components_register_nodes(${PROJECT_NAME} "prometheus_ros::TopicMonitorComponent")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

install(DIRECTORY launch parameters config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
