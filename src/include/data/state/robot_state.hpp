#pragma once

#include <Eigen/Dense>

#include "custom_msgs/msg/vision_msg.hpp"

/* State structure */
struct RobotState {
  double t{0.0};
  double dt{0.0};
  Eigen::Vector2d p{Eigen::Vector2d::Zero()};   // Position on image
  Eigen::Vector2d v{Eigen::Vector2d::Zero()};   // Velocity on image 
  Eigen::Vector2d a{Eigen::Vector2d::Zero()};   // Acceleration on image
  bool has_velocity{false};
  bool has_acceleration{false};
  bool detected{false};
  bool tracked{false};
  bool process{false};
  
  Eigen::Vector2d w{Eigen::Vector2d::Zero()};   // Angular velocity of camera - map into image velocity later

  RobotState() = default;
  RobotState(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
    t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    if (msg->p.size() >= 2) {
      p = Eigen::Vector2d(msg->p[0], msg->p[1]);
    }
    if (msg->v.size() >= 2) {
      v = Eigen::Vector2d(msg->v[0], msg->v[1]);
      has_velocity = true;
    }
    if (msg->a.size() >= 2) {
      a = Eigen::Vector2d(msg->a[0], msg->a[1]);
      has_acceleration = true;
    }

    detected = msg->detected;
    tracked = msg->tracked;
  }
};
