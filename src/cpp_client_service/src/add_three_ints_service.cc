#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <memory>

void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request, std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response> response) {
  // add three integers from the request and store result in repsonse
  response->sum = request->a + request->b + request-> c;
  // log to console
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld", request->a, request->b, request->c);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv) {
  // initialize ROS2 c++ client library
  rclcpp::init(argc, argv);
  // create node with the name "add_two_ints_server"
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");

  // create service and advertise it over network
  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service = node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

  // start the node, making service available for use
  rclcpp::spin(node);
  rclcpp::shutdown();
}
