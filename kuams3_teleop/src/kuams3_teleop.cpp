#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mutex>

class Kuams3Teleop : public rclcpp::Node
{
public:
    Kuams3Teleop() : Node("kuams3_teleop")
    {
        // Declare and get parameters
        this->declare_parameter<int>("axis_linear_x", 1);
        this->declare_parameter<int>("axis_linear_y", 2);
        this->declare_parameter<int>("axis_angular", 0);
        this->declare_parameter<int>("axis_deadman", 4);
        this->declare_parameter<double>("scale_linear", 0.3);
        this->declare_parameter<double>("scale_angular", 0.9);

        this->get_parameter("axis_linear_x", linear_x_axis_);
        this->get_parameter("axis_linear_y", linear_y_axis_);
        this->get_parameter("axis_angular", angular_axis_);
        this->get_parameter("axis_deadman", deadman_axis_);
        this->get_parameter("scale_linear", linear_scale_);
        this->get_parameter("scale_angular", angular_scale_);

        // Initialize flags
        deadman_pressed_ = false;
        zero_twist_published_ = false;

        // Create publisher and subscriber
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Kuams3Teleop::joyCallback, this, std::placeholders::_1));

        // Create a timer to publish at regular intervals
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Kuams3Teleop::publish, this));
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        auto vel = geometry_msgs::msg::Twist();
        vel.angular.z = angular_scale_ * joy->axes[angular_axis_];
        vel.linear.x = linear_scale_ * joy->axes[linear_x_axis_];
        vel.linear.y = linear_scale_ * joy->axes[linear_y_axis_];
        last_published_ = vel;
        deadman_pressed_ = joy->buttons[deadman_axis_];
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(publish_mutex_);

        if (deadman_pressed_)
        {
            vel_pub_->publish(last_published_);
            zero_twist_published_ = false;
        }
        else if (!deadman_pressed_ && !zero_twist_published_)
        {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            zero_twist_published_ = true;
        }
    }

    // Node parameters
    int linear_x_axis_, linear_y_axis_, angular_axis_, deadman_axis_;
    double linear_scale_, angular_scale_;

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Timers and mutex
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex publish_mutex_;

    // State variables
    geometry_msgs::msg::Twist last_published_;
    bool deadman_pressed_;
    bool zero_twist_published_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Kuams3Teleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
