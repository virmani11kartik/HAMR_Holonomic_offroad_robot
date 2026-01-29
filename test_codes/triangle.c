#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace std;
using namespace std::chrono_literals;

enum class State { MOVE_TO_CORNER, TURN_AT_CORNER, DONE };

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

class HamrTriangleNode : public rclcpp::Node {
public:
  HamrTriangleNode()
  : Node("hamr_triangle_node"),
    side_length_(declare_parameter("side_length_m", 4.0)),
    triangle_type_(declare_parameter("triangle_type", "equilateral")), // "equilateral", "right", "isosceles"
    max_linear_speed_(declare_parameter("max_linear_speed", 3.5)),      // m/s
    max_angular_speed_(declare_parameter("max_angular_speed", 8.0)),    // rad/s
    wheel_base_(declare_parameter("wheel_base_m", 0.16)),               // distance between wheels
    position_tolerance_(declare_parameter("position_tolerance_m", 0.4)),
    angle_tolerance_(deg2rad(declare_parameter("angle_tolerance_deg", 0.5))),
    k_linear_(declare_parameter("k_linear", 4.0)),                      // Linear position gain
    k_angular_(declare_parameter("k_angular", 10.0)),                   // Angular gain
    settle_time_s_(declare_parameter("settle_time_s", 0.2))
  {
    left_pub_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel", 1);
    right_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/hamr/odom", 40,
      [this](const nav_msgs::msg::Odometry::SharedPtr m){ this->on_odom(m); });

    timer_ = create_wall_timer(20ms, [this]{ this->tick(); });

    RCLCPP_INFO(this->get_logger(), "HAMR Triangle Node initialized (%s triangle). Waiting for odometry...", 
                triangle_type_.c_str());
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
      initialize_triangle();
    }
  }

  void initialize_triangle() {
    Point start(curr_x_, curr_y_);
    
    double cos_yaw = std::cos(curr_yaw_);
    double sin_yaw = std::sin(curr_yaw_);
    
    waypoints_.clear();
    target_headings_.clear();
    
    if (triangle_type_ == "equilateral") {
      initialize_equilateral_triangle(start, cos_yaw, sin_yaw);
    } else if (triangle_type_ == "right") {
      initialize_right_triangle(start, cos_yaw, sin_yaw);
    } else if (triangle_type_ == "isosceles") {
      initialize_isosceles_triangle(start, cos_yaw, sin_yaw);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown triangle type: %s. Using equilateral.", triangle_type_.c_str());
      initialize_equilateral_triangle(start, cos_yaw, sin_yaw);
    }

    current_waypoint_ = 0;
    state_ = State::MOVE_TO_CORNER;
    initialized_ = true;
    settle_timer_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Triangle initialized. Moving to first corner: (%.2f, %.2f)",
                waypoints_[0].x, waypoints_[0].y);
    
    // Log all waypoints for debugging
    for (size_t i = 0; i < waypoints_.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Waypoint %zu: (%.2f, %.2f)", i, waypoints_[i].x, waypoints_[i].y);
    }
  }

  void initialize_equilateral_triangle(const Point& start, double cos_yaw, double sin_yaw) {
    // Equilateral triangle: all sides equal, all angles 60°
    
    // First vertex: side_length_ distance forward
    waypoints_.push_back(Point(
      start.x + side_length_ * cos_yaw,
      start.y + side_length_ * sin_yaw
    ));
    
    // Second vertex: 60° left turn from first side
    double angle1 = curr_yaw_ + deg2rad(60.0);
    waypoints_.push_back(Point(
      waypoints_[0].x + side_length_ * std::cos(angle1),
      waypoints_[0].y + side_length_ * std::sin(angle1)
    ));
    
    // Third vertex: back to start
    waypoints_.push_back(start);
    
    // Target headings: 120° turns (exterior angles of equilateral triangle)
    target_headings_.push_back(normalize_angle(curr_yaw_ + deg2rad(120.0)));  // After first side
    target_headings_.push_back(normalize_angle(curr_yaw_ + deg2rad(240.0)));  // After second side
    target_headings_.push_back(normalize_angle(curr_yaw_));                    // After third side (back to start)
    
    RCLCPP_INFO(this->get_logger(), "Equilateral triangle created with side length %.2f m", side_length_);
  }

  void initialize_right_triangle(const Point& start, double cos_yaw, double sin_yaw) {
    // Right triangle: 90° angle at second vertex
    
    // First vertex: side_length_ distance forward
    waypoints_.push_back(Point(
      start.x + side_length_ * cos_yaw,
      start.y + side_length_ * sin_yaw
    ));
    
    // Second vertex: 90° left turn, side_length_ distance
    double angle1 = curr_yaw_ + deg2rad(90.0);
    waypoints_.push_back(Point(
      waypoints_[0].x + side_length_ * std::cos(angle1),
      waypoints_[0].y + side_length_ * std::sin(angle1)
    ));
    
    // Third vertex: back to start
    waypoints_.push_back(start);
    
    // Target headings for right triangle
    target_headings_.push_back(normalize_angle(curr_yaw_ + deg2rad(90.0)));   // 90° turn
    target_headings_.push_back(normalize_angle(curr_yaw_ + deg2rad(135.0)));  // ~135° turn (depends on triangle)
    target_headings_.push_back(normalize_angle(curr_yaw_));                   // Back to start orientation
    
    RCLCPP_INFO(this->get_logger(), "Right triangle created with legs of %.2f m", side_length_);
  }

  void initialize_isosceles_triangle(const Point& start, double cos_yaw, double sin_yaw) {
    // Isosceles triangle: two equal sides, apex angle of 60°
    
    double apex_angle = deg2rad(60.0);  // Angle at the top
    double base_angle = deg2rad(60.0);  // Base angles are equal: (180° - 60°) / 2 = 60°
    
    // First vertex: side_length_ distance forward
    waypoints_.push_back(Point(
      start.x + side_length_ * cos_yaw,
      start.y + side_length_ * sin_yaw
    ));
    
    // Second vertex (apex): turn by base_angle from first side
    double angle1 = curr_yaw_ + base_angle;
    waypoints_.push_back(Point(
      waypoints_[0].x + side_length_ * std::cos(angle1),
      waypoints_[0].y + side_length_ * std::sin(angle1)
    ));
    
    // Third vertex: back to start
    waypoints_.push_back(start);
    
    // Target headings
    double exterior_angle = M_PI - base_angle;  // Exterior angle = 180° - interior angle
    target_headings_.push_back(normalize_angle(curr_yaw_ + exterior_angle));
    target_headings_.push_back(normalize_angle(curr_yaw_ + 2 * exterior_angle));
    target_headings_.push_back(normalize_angle(curr_yaw_));
    
    RCLCPP_INFO(this->get_logger(), "Isosceles triangle created with equal sides of %.2f m", side_length_);
  }

  void tick() {
    if (!initialized_ || !have_odom_) {
      publish_zero();
      return;
    }

    switch (state_) {
      case State::MOVE_TO_CORNER:
        move_to_current_waypoint();
        break;
      case State::TURN_AT_CORNER:
        turn_to_target_heading();
        break;
      case State::DONE:
        publish_zero();
        break;
    }
  }

  void move_to_current_waypoint() {
    if (current_waypoint_ >= waypoints_.size()) {
      state_ = State::DONE;
      RCLCPP_INFO(this->get_logger(), "Triangle completed!");
      return;
    }

    const Point& target = waypoints_[current_waypoint_];
    
    // Calculate errors
    double dx = target.x - curr_x_;
    double dy = target.y - curr_y_;
    double distance = std::hypot(dx, dy);
    
    if (distance < position_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %d. Starting turn...", current_waypoint_);
      state_ = State::TURN_AT_CORNER;
      settle_timer_ = this->now();
      publish_zero();
      return;
    }

    // Calculate desired heading to target
    double desired_heading = std::atan2(dy, dx);
    double heading_error = normalize_angle(desired_heading - curr_yaw_);
    
    // Pure pursuit style control
    double linear_vel = std::min(max_linear_speed_, k_linear_ * distance);
    double angular_vel = k_angular_ * heading_error;
    
    // Reduce linear speed if we need to turn a lot
    if (std::fabs(heading_error) > deg2rad(45.0)) {
      linear_vel *= 0.3; // Slow down for sharp turns
    }
    
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    // Convert to differential drive
    convert_to_wheel_speeds(linear_vel, angular_vel);
  }

  void turn_to_target_heading() {
    if (current_waypoint_ >= target_headings_.size()) {
      current_waypoint_++;
      state_ = State::MOVE_TO_CORNER;
      return;
    }

    double target_heading = target_headings_[current_waypoint_];
    double heading_error = normalize_angle(target_heading - curr_yaw_);
    
    if (std::fabs(heading_error) < angle_tolerance_) {
      // Check if we've been settled for enough time
      if ((this->now() - settle_timer_).seconds() > settle_time_s_) {
        RCLCPP_INFO(this->get_logger(), "Turn completed. Moving to next waypoint...");
        current_waypoint_++;
        state_ = State::MOVE_TO_CORNER;
        return;
      }
      // Stay stopped while settling
      publish_zero();
      return;
    }

    // Reset settle timer if we're not in tolerance
    settle_timer_ = this->now();
    
    // Pure rotation
    double angular_vel = k_angular_ * heading_error;
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    // Minimum speed to overcome static friction
    double min_angular_speed = 0.2;
    if (std::fabs(angular_vel) < min_angular_speed) {
      angular_vel = std::copysign(min_angular_speed, angular_vel);
    }
    
    convert_to_wheel_speeds(0.0, angular_vel); // Pure rotation
  }

  void convert_to_wheel_speeds(double linear_vel, double angular_vel) {
    double wheel_vel_left = linear_vel - (angular_vel * wheel_base_) / 2.0;
    double wheel_vel_right = linear_vel + (angular_vel * wheel_base_) / 2.0;
    
    if (linear_vel == 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "TURN PHASE: ω_left=%.3f, ω_right=%.3f (angular=%.3f rad/s, base=%.3f m)",
        wheel_vel_left, wheel_vel_right, angular_vel, wheel_base_);
    }

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
  double side_length_;
  std::string triangle_type_;
  double max_linear_speed_;
  double max_angular_speed_;
  double wheel_base_;
  double position_tolerance_;
  double angle_tolerance_;
  double k_linear_;
  double k_angular_;
  double settle_time_s_;

  // ---- State ----
  bool initialized_{false};
  bool have_odom_{false};
  State state_{State::MOVE_TO_CORNER};
  
  std::vector<Point> waypoints_;
  std::vector<double> target_headings_;
  int current_waypoint_{0};
  rclcpp::Time settle_timer_;

  // ---- Odometry ----
  double curr_x_{0.0}, curr_y_{0.0}, curr_yaw_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HamrTriangleNode>());
  rclcpp::shutdown();
  return 0;
}
