#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace std;
using namespace std::chrono_literals;

enum class State { MOVE_TO_START, FOLLOW_CIRCLE, DONE };

struct Point {
  double x, y;
  Point(double x = 0, double y = 0) : x(x), y(y) {}
};

static double normalize_angle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

static double deg2rad(double d) { return d * M_PI / 180.0; }

class HamrCircleNode : public rclcpp::Node {
public:
  HamrCircleNode()
  : Node("hamr_circle_node"),
    radius_(declare_parameter("radius_m", 5.0)),
    max_linear_speed_(declare_parameter("max_linear_speed", 3.5)),      // m/s
    max_angular_speed_(declare_parameter("max_angular_speed", 8.0)),    // rad/s
    wheel_base_(declare_parameter("wheel_base_m", 0.16)),               // distance between wheels
    position_tolerance_(declare_parameter("position_tolerance_m", 0.4)),
    angle_tolerance_(deg2rad(declare_parameter("angle_tolerance_deg", 5.0))),
    k_linear_(declare_parameter("k_linear", 4.0)),                      // Linear position gain
    k_angular_(declare_parameter("k_angular", 10.0)),                   // Angular gain
    lookahead_distance_(declare_parameter("lookahead_distance_m", 1.0)), // Pure pursuit lookahead
    num_waypoints_(declare_parameter("num_waypoints", 36))              // 36 waypoints = 10° apart
  {
    left_pub_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel", 1);
    right_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/hamr/odom", 40,
      [this](const nav_msgs::msg::Odometry::SharedPtr m){ this->on_odom(m); });

    timer_ = create_wall_timer(20ms, [this]{ this->tick(); });

    RCLCPP_INFO(this->get_logger(), "HAMR Circle Node initialized. Waiting for odometry...");
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr &msg) {
    curr_x_ = msg->pose.pose.position.x;
    curr_y_ = msg->pose.pose.position.y;

    const auto &q = msg->pose.pose.orientation;
    const double s = 2.0 * (q.w * q.z + q.x * q.y);
    const double c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    curr_yaw_ = std::atan2(s, c);

    have_odom_ = true;

    if (!initialized_) {
      initialize_circle();
    }
  }

  void initialize_circle() {
    // Calculate circle center - place it radius_ distance to the left of starting position
    double cos_yaw = std::cos(curr_yaw_);
    double sin_yaw = std::sin(curr_yaw_);
    
    // Circle center is radius_ distance perpendicular to initial heading (left side)
    circle_center_x_ = curr_x_ - radius_ * sin_yaw;
    circle_center_y_ = curr_y_ + radius_ * cos_yaw;
    
    // Generate waypoints around the circle
    waypoints_.clear();
    for (int i = 0; i < num_waypoints_; i++) {
      double angle = 2.0 * M_PI * i / num_waypoints_;
      // Start from current position angle
      double start_angle = std::atan2(curr_y_ - circle_center_y_, curr_x_ - circle_center_x_);
      double waypoint_angle = start_angle + angle;
      
      Point waypoint(
        circle_center_x_ + radius_ * std::cos(waypoint_angle),
        circle_center_y_ + radius_ * std::sin(waypoint_angle)
      );
      waypoints_.push_back(waypoint);
    }
    
    current_waypoint_ = 0;
    state_ = State::MOVE_TO_START;
    initialized_ = true;
    laps_completed_ = 0;

    RCLCPP_INFO(this->get_logger(), 
                "Circle initialized. Center: (%.2f, %.2f), Radius: %.2f m, Moving to start...",
                circle_center_x_, circle_center_y_, radius_);
  }

  void tick() {
    if (!initialized_ || !have_odom_) {
      publish_zero();
      return;
    }

    switch (state_) {
      case State::MOVE_TO_START:
        move_to_start_position();
        break;
      case State::FOLLOW_CIRCLE:
        follow_circle_path();
        break;
      case State::DONE:
        publish_zero();
        break;
    }
  }

  void move_to_start_position() {
    const Point& target = waypoints_[0];
    
    // Calculate errors
    double dx = target.x - curr_x_;
    double dy = target.y - curr_y_;
    double distance = std::hypot(dx, dy);
    
    if (distance < position_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Reached circle start position. Beginning circular path...");
      state_ = State::FOLLOW_CIRCLE;
      return;
    }

    // Calculate desired heading to target
    double desired_heading = std::atan2(dy, dx);
    double heading_error = normalize_angle(desired_heading - curr_yaw_);
    
    // Control logic
    double linear_vel = std::min(max_linear_speed_, k_linear_ * distance);
    double angular_vel = k_angular_ * heading_error;
    
    // Reduce linear speed for sharp turns
    if (std::fabs(heading_error) > deg2rad(45.0)) {
      linear_vel *= 0.3;
    }
    
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    convert_to_wheel_speeds(linear_vel, angular_vel);
  }

  void follow_circle_path() {
    // Pure pursuit algorithm for circular path following
    
    // Find the target waypoint using lookahead distance
    int target_waypoint = find_lookahead_waypoint();
    
    if (target_waypoint == -1) {
      // Completed circle
      laps_completed_++;
      RCLCPP_INFO(this->get_logger(), "Completed lap %d", laps_completed_);
      
      // You can modify this condition to do multiple laps or stop after one
      if (laps_completed_ >= 1) {
        state_ = State::DONE;
        RCLCPP_INFO(this->get_logger(), "Circle navigation completed!");
        return;
      } else {
        // Reset for another lap
        current_waypoint_ = 0;
      }
    }

    const Point& target = waypoints_[target_waypoint];
    
    // Calculate errors
    double dx = target.x - curr_x_;
    double dy = target.y - curr_y_;
    double distance = std::hypot(dx, dy);
    
    // Calculate desired heading to target
    double desired_heading = std::atan2(dy, dx);
    double heading_error = normalize_angle(desired_heading - curr_yaw_);
    
    // For circular motion, we can also use a constant linear speed approach
    // with curvature-based angular velocity
    double linear_vel = max_linear_speed_ * 0.7; // Use 70% of max speed for smooth circle
    double angular_vel = k_angular_ * heading_error;
    
    // Alternative: Calculate curvature for perfect circular motion
    // double curvature = 1.0 / radius_;
    // double angular_vel = linear_vel * curvature;
    
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    convert_to_wheel_speeds(linear_vel, angular_vel);
    
    // Update current waypoint if we're close enough
    if (distance < position_tolerance_ * 2.0) { // Use larger tolerance for circle following
      current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
    }
  }

  int find_lookahead_waypoint() {
    double min_distance = std::numeric_limits<double>::max();
    int closest_waypoint = current_waypoint_;
    
    // Find closest waypoint
    for (int i = 0; i < waypoints_.size(); i++) {
      double dx = waypoints_[i].x - curr_x_;
      double dy = waypoints_[i].y - curr_y_;
      double distance = std::hypot(dx, dy);
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_waypoint = i;
      }
    }
    
    // Find waypoint at lookahead distance from closest waypoint
    for (int offset = 0; offset < waypoints_.size(); offset++) {
      int waypoint_idx = (closest_waypoint + offset) % waypoints_.size();
      double dx = waypoints_[waypoint_idx].x - curr_x_;
      double dy = waypoints_[waypoint_idx].y - curr_y_;
      double distance = std::hypot(dx, dy);
      
      if (distance >= lookahead_distance_) {
        return waypoint_idx;
      }
    }
    
    return -1; // Completed circle
  }

  void convert_to_wheel_speeds(double linear_vel, double angular_vel) {
    double wheel_vel_left = linear_vel - (angular_vel * wheel_base_) / 2.0;
    double wheel_vel_right = linear_vel + (angular_vel * wheel_base_) / 2.0;
    
    std_msgs::msg::Float64 L, R;
    L.data = wheel_vel_left;
    R.data = wheel_vel_right;
    
    publish(L, R);
  }

  void publish(const std_msgs::msg::Float64 &L, const std_msgs::msg::Float64 &R) {
    left_pub_->publish(L);
    right_pub_->publish(R);
  }

  void publish_zero() {
    std_msgs::msg::Float64 L, R;
    L.data = R.data = 0.0;
    publish(L, R);
  }

  // ---- ROS I/O ----
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Parameters ----
  double radius_;
  double max_linear_speed_;
  double max_angular_speed_;
  double wheel_base_;
  double position_tolerance_;
  double angle_tolerance_;
  double k_linear_;
  double k_angular_;
  double lookahead_distance_;
  int num_waypoints_;

  // ---- State ----
  bool initialized_{false};
  bool have_odom_{false};
  State state_{State::MOVE_TO_START};
  
  std::vector<Point> waypoints_;
  int current_waypoint_{0};
  int laps_completed_{0};
  
  // Circle parameters
  double circle_center_x_{0.0}, circle_center_y_{0.0};

  // ---- Odometry ----
  double curr_x_{0.0}, curr_y_{0.0}, curr_yaw_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HamrCircleNode>());
  rclcpp::shutdown();
  return 0;
}
