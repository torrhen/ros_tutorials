#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/contact.hpp"
// include custom message definition
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

// create publisher node for sending address book contacts
class AddressBookPublisher : public rclcpp::Node {
  public:
    AddressBookPublisher() : Node("address_book_publisher") {
      // create publisher for /address_book topic
      address_book_publisher_ = this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

      // callback to periodically publish on /address_book
      auto publish_msg = [this]() -> void {
        // create message to publish
        auto message = more_interfaces::msg::AddressBook();
	{
          tutorial_interfaces::msg::Contact contact;
          contact.first_name = "John";
          contact.last_name = "Smith";
          contact.phone_number = "123456789";
          contact.phone_type = contact.PHONE_TYPE_MOBILE;
          // add contact to address book
          message.address_book.push_back(contact);
        }
        {
          tutorial_interfaces::msg::Contact contact;
          contact.first_name = "Jane";
          contact.last_name = "Doe";
          contact.phone_number = "987654321";
          contact.phone_type = contact.PHONE_TYPE_WORK;
          // add contact to address book
          message.address_book.push_back(contact);
        }
        std::cout << "Publishing address book..." << std::endl;
        for (auto contact : message.address_book) {
          std::cout << "Publishing contact:\n" << contact.first_name << " " << contact.last_name << std::endl;
        }
        // publish message
        this->address_book_publisher_->publish(message);
      };

      // timer to publish message every second
      timer_ = this->create_wall_timer(1s, publish_msg);
    }
  
  private:
    rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}
