#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp" // package dependency

using namespace std;

class MinimalParam : public rclcpp::Node {
    public:
        MinimalParam() : Node("minimal_param_node") {
            // OPTIONAL parameter description
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Example parameter.";

            // create node parameter with default value, infer type
            this->declare_parameter("my_parameter", "world", param_desc);
            // excecute callback every second
            timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalParam::timer_callback, this));

        }

        void timer_callback() {
            std::string my_param = this->get_parameter("my_parameter").as_string();
            RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());
            // reset parameter to the default value
            std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
            this->set_parameters(all_new_parameters);
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
}
