cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) # build time dependency
find_package(tutorial_interfaces REQUIRED) # build time dependency
find_package(rclcpp REQUIRED)

# declare list of messages to generate
set (msg_files "msg/AddressBook.msg")

# generate messages
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES tutorial_interfaces)

# export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

add_executable(publish_address_book src/publish_address_book.cc)
add_executable(subscribe_address_book src/subscribe_address_book.cc)

ament_target_dependencies(publish_address_book rclcpp)
ament_target_dependencies(subscribe_address_book rclcpp)

install(TARGETS publish_address_book subscribe_address_book DESTINATION lib/${PROJECT_NAME})

# find the relevant c++ generate from AddressBook.msg and link to target
# ** NOTE: required only when using an interface from the same package the interface was defined in **
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(publish_address_book "${cpp_typesupport_target}")
target_link_libraries(subscribe_address_book "${cpp_typesupport_target}")

ament_package()
