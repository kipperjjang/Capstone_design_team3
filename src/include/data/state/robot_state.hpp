#pragma once

#include <Eigen/Dense>

#include <string>

#include "custom_msgs/msg/vision_msg.hpp"

/* State structure */
struct RobotState {
  double t{0.0};
  double dt{0.0};

  // Image pixel data (observed data)
  Eigen::Vector2d p{Eigen::Vector2d::Zero()};           // Position on image
  
  Eigen::Vector2d bbox{Eigen::Vector2d::Zero()};        // Size of bbox
  Eigen::Vector2d img_center{Eigen::Vector2d::Zero()};  // Image center
  std::string camera{"webcam"};

  // Yaw Pitch of the real robot (observed data)
  Eigen::Vector2d joint{Eigen::Vector2d::Zero()};       // Robot joint angle
  Eigen::Vector2d joint_vel{Eigen::Vector2d::Zero()};   // Robot joint velocity

  // Bool
  bool detected{false};
  
  Eigen::Vector2d w{Eigen::Vector2d::Zero()};   // Angular velocity of camera - map into image velocity later

  RobotState() = default;
  RobotState(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
    t = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    if (msg->p.size() >= 2) {
      p = Eigen::Vector2d(msg->p[0], msg->p[1]);
    }

    if (msg->bbox.size() >= 2) {
      bbox = Eigen::Vector2d(msg->bbox[0], msg->bbox[1]);
    }
    if (msg->img_center.size() >= 2) {
      img_center = Eigen::Vector2d(msg->img_center[0], msg->img_center[1]);
    }
    if (!msg->camera.empty()) {
      camera = msg->camera;
    }
    detected = msg->detected;
  }
};
