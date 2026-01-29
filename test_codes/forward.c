#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>

using namespace std;
using namespace std_msgs::msg;
using namespace std::chrono_literals;

class starightnode: public rclcpp::Node
{
    public:
        starightnode()
        : Node("straight_node"),
            speed_rad_s(declare_parameter("wheel_speed_rad_s",5.0)),
            stop_distance(declare_parameter("stop_distance_m",5.0)),
            have_start(false), stopped(false)
        {
            left_pub = create_publisher<Float64>("/left_wheel/cmd_vel",1);
            right_pub = create_publisher<Float64>("/right_wheel/cmd_vel",1);
            odom_sub = create_subscription<nav_msgs::msg::Odometry>("/hamr/odom", 10,[this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->odom_cb(msg); });
            timer = create_wall_timer(100ms, [this]{this->tick();});

        }
    private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;

        if (!have_start) {
            x0= x; y0 = y;
            have_start = true;
            RCLCPP_INFO(get_logger(), "Start pose latched at (%.3f, %.3f).", x0, y0);
            return;
        }

        const double dx = x - x0;
        const double dy = y - y0;
        const double dist = std::hypot(dx, dy);

        if (!stopped && dist >= stop_distance) {
            stopped = true;
            RCLCPP_INFO(get_logger(), "Target distance reached: %.3f m (threshold %.3f m). Stopping.", dist, stop_distance);
        }
    }

    void tick()
    {
        std_msgs::msg::Float64 left, right;

        if (!have_start) {
            // Haven't seen odom yet—publish zeros to be safe
            left.data = 0.0; right.data = 0.0;
        } else if (!stopped) {
            // Drive straight with equal wheel speeds
            left.data = speed_rad_s;
            right.data = speed_rad_s;
        } else {
            // Stopped state
            left.data = 0.0; right.data = 0.0;
        }

        left_pub->publish(left);
        right_pub->publish(right);

    }

    rclcpp::Publisher<Float64>::SharedPtr left_pub, right_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer;

    //params and state
    double speed_rad_s;
    double stop_distance;
    bool have_start, stopped;
    double x0{0.0}, y0{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<starightnode>());
    rclcpp::shutdown();
    return 0;
}
