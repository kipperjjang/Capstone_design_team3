#pragma once

#include <Eigen/Dense>

#include "custom_msgs/msg/vision_msg.hpp"

/* State structure */
struct RobotState {
  double t{0.0};
  double dt{0.0};

  // Image pixel data (observed data)
  Eigen::Vector2d p{Eigen::Vector2d::Zero()};           // Position on image
  Eigen::Vector2d v{Eigen::Vector2d::Zero()};           // Velocity on image
  Eigen::Vector2d a{Eigen::Vector2d::Zero()};           // Acceleration on image

  // Yaw Pitch of the real robot (observed data)
  Eigen::Vector2d joint{Eigen::Vector2d::Zero()};       // Robot joint angle
  Eigen::Vector2d joint_vel{Eigen::Vector2d::Zero()};   // Robot joint velocity

  // Yaw Pitch angle of the bell (nominal data)
  Eigen::Vector2d angle{Eigen::Vector2d::Zero()};       // Bell angle
  Eigen::Vector2d omega{Eigen::Vector2d::Zero()};       // Bell velocity

  // Error
  Eigen::Vector2d error{Eigen::Vector2d::Zero()};       // Error; updated during control

  // Jacobian matrix for omega computation
  // Eigen::Matrix2d Jp{Eigen::Matrix2d::Zero()};          // Pixel->bell jacobian
  // Eigen::Matrix2d Jj{Eigen::Matrix2d::Zero()};          // Joint->bell jacobian

  // Bool
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

  void setJoint(const Eigen::Vector2d &j, const Eigen::Vector2d &jv) {
    joint = j;
    joint_vel = jv;
  }
};
