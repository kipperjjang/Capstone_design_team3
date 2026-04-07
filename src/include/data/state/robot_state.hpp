#pragma once

#include <Eigen/Dense>
#include "custom_msgs/msg/vision_msg.hpp"

/* State structure */
struct RobotState {
  double t{0.0};
  double dt{0.0};
  Eigen::Vector2d p{Eigen::Vector2d::Zero()};   // Position on image
  Eigen::Vector2d v{Eigen::Vector2d::Zero()};   // Velocity on image 
  bool detected{false};
  bool tracked{false};
  bool process{false};
  
  Eigen::Vector2d w{Eigen::Vector2d::Zero()};   // Angular velocity of camera - map into image velocity later

  RobotState(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
    t = msg->stamp.seconds();
    p = msg->p;
    v = msg->v;
    detected = msg->detected;
    tracked = msg->tracked;
  }
};

