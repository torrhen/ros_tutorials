#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/contact.hpp"
#include "more_interfaces/msg/address_book.hpp"

using std::placeholders::_1;

class AddressBookSubscriber : public rclcpp::Node {
  public:
    AddressBookSubscriber()
    : Node("address_book_subscriber") {
      subscription_ = this->create_subscription<more_interfaces::msg::AddressBook>("address_book", 10, std::bind(&AddressBookSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const more_interfaces::msg::AddressBook &msg) const {
      for (const auto& contact: msg.address_book) {
      RCLCPP_INFO(this->get_logger(), "I heard: first name: '%s' last name: '%s', phone number: '%s' phone type: '%d'", contact.first_name.c_str(), contact.last_name.c_str(), contact.phone_number.c_str(), contact.phone_type);
      }
    }
    rclcpp::Subscription<more_interfaces::msg::AddressBook>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookSubscriber>());
  rclcpp::shutdown();
  return 0;
}
