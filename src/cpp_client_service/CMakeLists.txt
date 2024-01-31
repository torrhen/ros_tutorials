cmake_minimum_required(VERSION 3.8)
project(cpp_client_service)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(service src/add_three_ints_service.cc)
ament_target_dependencies(service rclcpp tutorial_interfaces)

add_executable(client src/add_three_ints_client.cc)
ament_target_dependencies(client rclcpp tutorial_interfaces)

install(TARGETS service client DESTINATION lib/${PROJECT_NAME})

ament_package()
