#pragma once

#include <Eigen/Dense>
#include <vector>
#include "geometry_msgs/msg/vector3.hpp"

template<typename T>
inline Eigen::VectorXd toEigen(const std::vector<T> &v) {
  return Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>(v.data(), v.size()).template cast<double>();
}

inline Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 &v) {
  return Eigen::Vector3d(v.x, v.y, v.z);
}

inline std::vector<double> toStdVector(const Eigen::VectorXd &v) {
  return std::vector<double>(v.data(), v.data() + v.size());
}
