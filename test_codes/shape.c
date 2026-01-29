#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

using namespace std;
using namespace std::chrono_literals;

enum class State { MOVE_TO_TARGET, TURN_AT_CORNER, MOVING_IN_CIRCLE, DONE };
enum class ShapeType { SQUARE, TRIANGLE, CIRCLE };

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

class HamrShapeNode : public rclcpp::Node {
public:
  HamrShapeNode()
  : Node("hamr_shape_node"),
    side_length_(declare_parameter("side_length_m", 3.0)),
    max_linear_speed_(declare_parameter("max_linear_speed", 3.5)),      // m/s
    max_angular_speed_(declare_parameter("max_angular_speed", 8.0)),    // rad/s
    wheel_base_(declare_parameter("wheel_base_m", 0.16)),               // distance between wheels
    position_tolerance_(declare_parameter("position_tolerance_m", 0.4)),
    angle_tolerance_(deg2rad(declare_parameter("angle_tolerance_deg", 0.5))),
    k_linear_(declare_parameter("k_linear", 4.0)),                      // Linear position gain
    k_angular_(declare_parameter("k_angular", 10.0)),                   // Angular gain
    settle_time_s_(declare_parameter("settle_time_s", 0.2)),
    circle_points_(declare_parameter("circle_points", 20))              // Points for circle approximation
  {
    // Set shape directly in code - change this line to select shape
    shape_type_ = ShapeType::CIRCLE;  // Options: SQUARE, TRIANGLE, CIRCLE
    
    // Get string for logging
    std::string shape_str = (shape_type_ == ShapeType::TRIANGLE) ? "triangle" :
                           (shape_type_ == ShapeType::CIRCLE) ? "circle" : "square";

    left_pub_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel", 1);
    right_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/hamr/odom", 40,
      [this](const nav_msgs::msg::Odometry::SharedPtr m){ this->on_odom(m); });

    timer_ = create_wall_timer(20ms, [this]{ this->tick(); });

    RCLCPP_INFO(this->get_logger(), "HAMR Shape Node initialized (%s). Waiting for odometry...", 
                shape_str.c_str());
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
      initialize_shape();
    }
  }

  void initialize_shape() {
    Point start(curr_x_, curr_y_);
    double cos_yaw = std::cos(curr_yaw_);
    double sin_yaw = std::sin(curr_yaw_);
    
    waypoints_.clear();
    target_headings_.clear();

    switch (shape_type_) {
      case ShapeType::SQUARE:
        initialize_square(start, cos_yaw, sin_yaw);
        state_ = State::MOVE_TO_TARGET;
        break;
      case ShapeType::TRIANGLE:
        initialize_triangle(start, cos_yaw, sin_yaw);
        state_ = State::MOVE_TO_TARGET;
        break;
      case ShapeType::CIRCLE:
        initialize_circle(start, cos_yaw, sin_yaw);
        state_ = State::MOVING_IN_CIRCLE;
        break;
    }

    current_waypoint_ = 0;
    initialized_ = true;
    settle_timer_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Shape initialized. Moving to first target: (%.2f, %.2f)",
                waypoints_[0].x, waypoints_[0].y);
  }

  void initialize_square(const Point& start, double cos_yaw, double sin_yaw) {
    // Same as your original square logic
    waypoints_.push_back(Point(
      start.x + side_length_ * cos_yaw,
      start.y + side_length_ * sin_yaw
    ));
    waypoints_.push_back(Point(
      start.x + side_length_ * cos_yaw - side_length_ * sin_yaw,
      start.y + side_length_ * sin_yaw + side_length_ * cos_yaw
    ));
    waypoints_.push_back(Point(
      start.x - side_length_ * sin_yaw,
      start.y + side_length_ * cos_yaw
    ));
    waypoints_.push_back(start); // Return to start
    
    // Target headings at each corner (90 degree turns)
    for (int i = 0; i < 3; i++) {
      target_headings_.push_back(normalize_angle(curr_yaw_ + (i + 1) * M_PI / 2));
    }
  }

  void initialize_triangle(const Point& start, double cos_yaw, double sin_yaw) {
    // Equilateral triangle with side_length_
    double height = side_length_ * std::sqrt(3.0) / 2.0;
    
    // First vertex: forward from start
    waypoints_.push_back(Point(
      start.x + side_length_ * cos_yaw,
      start.y + side_length_ * sin_yaw
    ));
    
    // Second vertex: 120 degrees left from first side
    double angle1 = curr_yaw_ + 2.0 * M_PI / 3.0; // 120 degrees
    waypoints_.push_back(Point(
      waypoints_[0].x + side_length_ * std::cos(angle1),
      waypoints_[0].y + side_length_ * std::sin(angle1)
    ));
    
    // Back to start
    waypoints_.push_back(start);
    
    // Target headings at each corner (120 degree turns)
    target_headings_.push_back(normalize_angle(curr_yaw_ + 2.0 * M_PI / 3.0)); // 120°
    target_headings_.push_back(normalize_angle(curr_yaw_ + 4.0 * M_PI / 3.0)); // 240°
  }

  void initialize_circle(const Point& start, double cos_yaw, double sin_yaw) {
    // Create circle as series of waypoints
    // Circle radius is side_length_, center is to the left of robot's initial heading
    Point center(
      start.x - side_length_ * sin_yaw,  // Left of robot
      start.y + side_length_ * cos_yaw
    );
    
    // Generate waypoints around the circle
    for (int i = 0; i < circle_points_; i++) {
      double angle = curr_yaw_ + (2.0 * M_PI * i) / circle_points_;
      waypoints_.push_back(Point(
        center.x + side_length_ * std::cos(angle),
        center.y + side_length_ * std::sin(angle)
      ));
    }
    
    // For circle, we don't use discrete turns, just smooth following
    circle_center_ = center;
  }

  void tick() {
    if (!initialized_ || !have_odom_) {
      publish_zero();
      return;
    }

    switch (state_) {
      case State::MOVE_TO_TARGET:
        move_to_current_waypoint();
        break;
      case State::TURN_AT_CORNER:
        turn_to_target_heading();
        break;
      case State::MOVING_IN_CIRCLE:
        move_in_circle();
        break;
      case State::DONE:
        publish_zero();
        break;
    }
  }

  void move_to_current_waypoint() {
    if (current_waypoint_ >= waypoints_.size()) {
      state_ = State::DONE;
      RCLCPP_INFO(this->get_logger(), "Shape completed!");
      return;
    }

    const Point& target = waypoints_[current_waypoint_];
    
    // Calculate errors
    double dx = target.x - curr_x_;
    double dy = target.y - curr_y_;
    double distance = std::hypot(dx, dy);
    
    if (distance < position_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %d. Starting turn...", current_waypoint_);
      
      // For shapes with corners, turn at each waypoint
      if (shape_type_ != ShapeType::CIRCLE && current_waypoint_ < target_headings_.size()) {
        state_ = State::TURN_AT_CORNER;
        settle_timer_ = this->now();
        publish_zero();
        return;
      } else {
        // No turn needed, go to next waypoint
        current_waypoint_++;
        return;
      }
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

  void move_in_circle() {
    // For circle: constant angular velocity to maintain circular motion
    // Calculate desired position on circle
    double time_elapsed = (this->now() - settle_timer_).seconds();
    double angular_progress = (time_elapsed * max_linear_speed_) / side_length_; // rad traveled
    
    // Check if we've completed the circle
    if (angular_progress > 2.0 * M_PI) {
      state_ = State::DONE;
      RCLCPP_INFO(this->get_logger(), "Circle completed!");
      return;
    }
    
    // Simple circular motion: constant forward speed + constant angular velocity
    double linear_vel = max_linear_speed_ * 0.6; // Moderate speed
    double angular_vel = linear_vel / side_length_; // v = ωr, so ω = v/r
    
    // Ensure we don't exceed limits
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    convert_to_wheel_speeds(linear_vel, angular_vel);
  }

  void turn_to_target_heading() {
    if (current_waypoint_ >= target_headings_.size()) {
      current_waypoint_++;
      state_ = State::MOVE_TO_TARGET;
      return;
    }

    double target_heading = target_headings_[current_waypoint_];
    double heading_error = normalize_angle(target_heading - curr_yaw_);
    
    if (std::fabs(heading_error) < angle_tolerance_) {
      // Check if we've been settled for enough time
      if ((this->now() - settle_timer_).seconds() > settle_time_s_) {
        RCLCPP_INFO(this->get_logger(), "Turn completed. Moving to next waypoint...");
        current_waypoint_++;
        state_ = State::MOVE_TO_TARGET;
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
  double max_linear_speed_;
  double max_angular_speed_;
  double wheel_base_;
  double position_tolerance_;
  double angle_tolerance_;
  double k_linear_;
  double k_angular_;
  double settle_time_s_;
  int circle_points_;
  
  // ---- Shape ----
  ShapeType shape_type_;
  Point circle_center_;

  // ---- State ----
  bool initialized_{false};
  bool have_odom_{false};
  State state_{State::MOVE_TO_TARGET};
  
  std::vector<Point> waypoints_;
  std::vector<double> target_headings_;
  int current_waypoint_{0};
  rclcpp::Time settle_timer_;

  // ---- Odometry ----
  double curr_x_{0.0}, curr_y_{0.0}, curr_yaw_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HamrShapeNode>());
  rclcpp::shutdown();
  return 0;
}
