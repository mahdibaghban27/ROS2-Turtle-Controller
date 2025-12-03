#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class UINode : public rclcpp::Node
{
public:
    UINode() : Node("ui_node")
    {
        RCLCPP_INFO(this->get_logger(), "UI Node started.");

        // Publisher to tell DistanceNode which turtle is currently moving
        active_pub_ = this->create_publisher<std_msgs::msg::String>("active_turtle", 10);

        // Timer to repeatedly check user input (non-blocking to ROS)
        timer_ = this->create_wall_timer(500ms, std::bind(&UINode::loop, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;

    bool waiting_input_ = true;
    std::string selected_turtle_;
    double linear_{0.0}, angular_{0.0};

    void loop()
    {
        if (!waiting_input_) {
            return;  // still sending previous command
        }

        // -------------------------------
        // 1) Ask for turtle selection
        // -------------------------------
        std::cout << "\nSelect turtle (turtle1 / turtle2): ";
        std::cin >> selected_turtle_;

        // Validate turtle name
        if (selected_turtle_ != "turtle1" && selected_turtle_ != "turtle2") {
            std::cout << "Invalid turtle name. Please type 'turtle1' or 'turtle2'." << std::endl;
            return; 
        }

        // Inform distance node about which turtle is about to move
        std_msgs::msg::String active_msg;
        active_msg.data = selected_turtle_;
        active_pub_->publish(active_msg);

        // -------------------------------
        // 2) Ask for linear and angular velocity
        // -------------------------------
        std::cout << "Enter linear velocity: ";
        std::cin >> linear_;

        std::cout << "Enter angular velocity: ";
        std::cin >> angular_;

        // -------------------------------
        // 3) Create publisher for selected turtle
        // -------------------------------
        std::string topic_name = "/" + selected_turtle_ + "/cmd_vel";
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);

        // Now execute the command for 1 second
        send_command();
    }

    void send_command()
    {
        waiting_input_ = false;

        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_;
        msg.angular.z = angular_;

        // Publish movement for 1 second
        auto start = this->now();
        rclcpp::Rate rate(10); // 10Hz = every 100ms

        while (rclcpp::ok() && (this->now() - start) < rclcpp::Duration(1, 0)) {
            pub_->publish(msg);
            rate.sleep();
        }

        // --------------------------------
        // Stop the turtle after the motion
        // --------------------------------
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        pub_->publish(msg);

        // Tell distance node that no turtle is moving anymore
        std_msgs::msg::String inactive_msg;
        inactive_msg.data = "none";
        active_pub_->publish(inactive_msg);

        waiting_input_ = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}