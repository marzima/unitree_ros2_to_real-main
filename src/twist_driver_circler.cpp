#include <rclcpp/rclcpp.hpp>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_unitree_legged_msgs_master/msg/high_cmd.h"
#include "ros2_unitree_legged_msgs_master/msg/high_state.h"
#include "ros2_unitree_legged_msgs_master/msg/imu.h"
#include "convert.h"

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

// If we remove the LCM layer, we would not need this global variables.
// We tried to put them in the class but it was not working.
UNITREE_LEGGED_SDK::LCM lcm_interface(UNITREE_LEGGED_SDK::HIGHLEVEL);
UNITREE_LEGGED_SDK::HighCmd high_cmd_lcm = {0};

// This class allows us to drive the Unitree A1 robot with twist message
class TwistDriverCircle : public rclcpp::Node
{
public:
    TwistDriverCircle() : Node("a1_twist_driver_circle")
    {
        // ROS parameters
        this->declare_parameter("start_walking", false);
        this->declare_parameter("using_imu_publisher", false);
        is_walking_ = this->get_parameter("start_walking").as_bool();
        using_imu_publisher = this->get_parameter("using_imu_publisher").as_bool();
        stop_walking_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&TwistDriverCircle::stopWalking, this));


        
        // Initilize publishers
        high_state_pub = this->create_publisher<ros2_unitree_legged_msgs_master::msg::HighState>("state", 10);
        if (using_imu_publisher)
            imu_pub = this->create_publisher<ros2_unitree_legged_msgs_master::msg::IMU>("imu", 10);
        
        // Initilize services
        change_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/change_mode",
            std::bind(&TwistDriverCircle::changeMode, this, std::placeholders::_1, std::placeholders::_2));
    }

    // This loop allows us to get robot data through LCM communication layer
    static void *lcm_update_loop(void *param)
    {
        UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
        while (rclcpp::ok)
        {
            data->Recv();
            usleep(2000);
        }
    }

    // Declare publishers
    rclcpp::Publisher<ros2_unitree_legged_msgs_master::msg::HighState>::SharedPtr high_state_pub;
    rclcpp::Publisher<ros2_unitree_legged_msgs_master::msg::IMU>::SharedPtr imu_pub;
    
    // Declare subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subs_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_mode_srv_;

    // Declare general attributs
    bool using_imu_publisher = false;

void driver()
    {
        ros2_unitree_legged_msgs_master::msg::HighCmd ros_high_cmd;

        if (is_walking_)
        {
            ros_high_cmd.mode = 2;
            ros_high_cmd.forward_speed = -0.1;
            ros_high_cmd.side_speed = 0.015;
            ros_high_cmd.rotate_speed = 0;
            //ros_high_cmd.side_speed = msg->linear.y;
            //ros_high_cmd.rotate_speed = msg->angular.z;
            //ros_high_cmd.pitch = msg->angular.y;
        }
        //else
        //{
          //  ros_high_cmd.mode = 1;
           // ros_high_cmd.yaw = msg->angular.z;
            //ros_high_cmd.roll = msg->linear.y;
            //ros_high_cmd.pitch = msg->angular.y;
            //ros_high_cmd.body_height = msg->linear.x;
        //}
        else
    {
        ros_high_cmd.mode = 1;
        // Implement the logic to stop walking here
        ros_high_cmd.forward_speed = 0.0; // Set forward speed to 0 to stop walking
    }

        high_cmd_lcm = ToLcm(ros_high_cmd, high_cmd_lcm);
        lcm_interface.Send(high_cmd_lcm);
    }

    // At this moment, there is not a way to change to sport mode in the SDK.
    // So this function only change the mode from walking to standing up without walking.
   
private:
    // Timer to stop walking after 5 seconds
    rclcpp::TimerBase::SharedPtr stop_walking_timer_;
    void stopWalking()
        {
            // Implement the logic to stop walking here
            // For example:
            is_walking_ = false;
            RCLCPP_INFO(this->get_logger(), "Stopped walking.");
        }
        // This function allows us to drive the robot in any mode
    
    void changeMode(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        ros2_unitree_legged_msgs_master::msg::HighCmd ros_high_cmd;

        if (is_walking_)
        {
            ros_high_cmd.mode = 1;
            is_walking_ = false;
        }
        else
        {
            ros_high_cmd.mode = 2;
            is_walking_ = true;
        }

        high_cmd_lcm = ToLcm(ros_high_cmd, high_cmd_lcm);
        lcm_interface.Send(high_cmd_lcm);
    }

    // Declare general attributs
    bool is_walking_ = false;
};

int main(int argc, char *argv[])
{
    // ROS2 Setup
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistDriverCircle>();
    rclcpp::WallRate loop_rate(500);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    ros2_unitree_legged_msgs_master::msg::HighCmd SendHighROS;
    ros2_unitree_legged_msgs_master::msg::HighState high_state_ros;

    // LCM Setup
    UNITREE_LEGGED_SDK::HighState high_state_lcm = {0};
    //UNITREE_LEGGED_SDK::LowState low_state_ros = {0};
    lcm_interface.SubscribeState();
    //state_interface.SubribeState();

    // Threads setup
    pthread_t tid;
    pthread_t tid2;
    pthread_create(&tid, NULL, node->lcm_update_loop, &lcm_interface);

    // ROS loop 
    while (rclcpp::ok())
    {
        lcm_interface.Get(high_state_lcm);
        high_state_ros = ToRos(high_state_lcm);

        // Publish robot state
        node->high_state_pub->publish(high_state_ros);

        if (node->using_imu_publisher)
            node->imu_pub->publish(high_state_ros.imu);

        

        node->driver();

        executor.spin_some();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
