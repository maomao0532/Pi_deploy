#pragma once

#include "robot_pai_controller/Types.h"

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
  // clang-format on
  return rotationMatrix;
}

template <typename T>
T square(T a) {
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}