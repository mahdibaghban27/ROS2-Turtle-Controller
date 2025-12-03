#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DistanceNode : public rclcpp::Node
{
public:
    DistanceNode() : Node("distance_node")
    {
        // Subscribe to both turtles' poses
        sub_t1_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
            std::bind(&DistanceNode::callbackT1, this, std::placeholders::_1));

        sub_t2_ = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10,
            std::bind(&DistanceNode::callbackT2, this, std::placeholders::_1));

        // Listen to which turtle is currently controlled
        active_sub_ = this->create_subscription<std_msgs::msg::String>("active_turtle", 10,
            std::bind(&DistanceNode::active_callback, this, std::placeholders::_1));

        // Publish distance between turtles
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("turtles_distance", 10);

        // Publishers to stop turtles if needed
        pub_t1_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pub_t2_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        last_active_ = "none";  // Start with no active turtle

        RCLCPP_INFO(this->get_logger(), "Distance node started.");
    }

private:
    turtlesim::msg::Pose t1_, t2_;
    bool t1_received_ = false;
    bool t2_received_ = false;
    std::string last_active_ = "none";

    // Time to rate-limit the warning message (avoid spam)
    rclcpp::Time last_warn_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t1_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t2_vel_;

    void active_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        last_active_ = msg->data;
    }

    void callbackT1(const turtlesim::msg::Pose::SharedPtr msg)
    {
        t1_ = *msg;
        t1_received_ = true;
        compute_logic();
    }

    void callbackT2(const turtlesim::msg::Pose::SharedPtr msg)
    {
        t2_ = *msg;
        t2_received_ = true;
        compute_logic();
    }

    void compute_logic()
    {
        if (!t1_received_ || !t2_received_) return;

        // Calculate distance
        float dx = t1_.x - t2_.x;
        float dy = t1_.y - t2_.y;
        float dist = std::sqrt(dx*dx + dy*dy);

        // Publish current distance
        std_msgs::msg::Float32 dist_msg;
        dist_msg.data = dist;
        dist_pub_->publish(dist_msg);

        // Slightly larger threshold so we stop before they actually touch (works perfectly even at high speed)
        const float threshold = 1.3f;

        if (dist < threshold)
        {
            // Warn only once per second
            auto now = this->now();
            if ((now - last_warn_time_) >= rclcpp::Duration::from_seconds(1.0))
            {
                RCLCPP_WARN(this->get_logger(), "Turtles too close (dist=%.2f)! Stopping the moving turtle...", dist);
                last_warn_time_ = now;
            }

            // Stop only the turtle that is currently being controlled
            if (last_active_ == "turtle1")
                stop_turtle(pub_t1_vel_);
            else if (last_active_ == "turtle2")
                stop_turtle(pub_t2_vel_);
        }

        // Boundary checks - only stop the moving turtle's forward movement
        if (t1_.x <= 1.0f || t1_.x >= 10.0f || t1_.y <= 1.0f || t1_.y >= 10.0f)
            if (last_active_ == "turtle1") stop_linear(pub_t1_vel_);

        if (t2_.x <= 1.0f || t2_.x >= 10.0f || t2_.y <= 1.0f || t2_.y >= 10.0f)
            if (last_active_ == "turtle2") stop_linear(pub_t2_vel_);
    }

    // Full stop (linear + angular) - used when too close
    void stop_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        pub->publish(stop);
    }

    // Only stop forward/backward movement near walls (allow rotation)
    void stop_linear(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        pub->publish(stop);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}