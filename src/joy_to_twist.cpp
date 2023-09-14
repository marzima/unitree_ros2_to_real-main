#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>
using std::placeholders::_1;

// This class allows us to convert joy message to twist message in order to drive
// the Unitree A1 robot
class JoyToTwist : public rclcpp::Node
{
  public:
    JoyToTwist() : Node("a1_joy_driver")
    {
        // Initialize publishers, subscribers and services
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        joy_subs_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyToTwist::joy_to_twist, this, _1));
        change_mode_client_ = this->create_client<std_srvs::srv::Trigger>("/change_mode");
        
        // Wait for the service to be available
        while (!change_mode_client_->wait_for_service(std::chrono::seconds(1)))
        {
          if (!rclcpp::ok())
          {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
          }
          RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
    }

  private:
    // This function allows us to convert joy message to twist message.
    // The mapping is the same that the controller provided by unitree.
    // But, this was tested with a PS4 controller, so other usb controller could
    // have a different hardware mapping.
    void joy_to_twist(const sensor_msgs::msg::Joy::SharedPtr joyMessage)
    {
        auto twistMessage = geometry_msgs::msg::Twist();
        twistMessage.angular.z = joyMessage->axes[left_joystick_x_axis_];
        twistMessage.linear.x = joyMessage->axes[left_joystick_y_axis_];
        twistMessage.linear.y = joyMessage->axes[right_joystick_x_axis_];
        twistMessage.angular.y = joyMessage->axes[right_joystick_y_axis_];

        // Using options button to change mode 
        if (joyMessage->buttons[options_button_] == 1) {
          // Prepare the request (empty request, nothing to add)
          auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

          // Call the service
          auto future = change_mode_client_->async_send_request(request);
        }

        twist_pub_->publish(twistMessage);
    }

    // Declare publishers, subscribers and services
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subs_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_mode_client_;

    // PS4 controller mapping
    int left_joystick_y_axis_ = 1;
    int left_joystick_x_axis_ = 0;
    int right_joystick_y_axis_ = 4;
    int right_joystick_x_axis_ = 3;
    int options_button_ = 9;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToTwist>());
  rclcpp::shutdown();
  return 0;
}