/*
 * eigen_conversions.h
 *
 *  Created on: Jun 22, 2012
 *      Author: acmarkus
 */

#ifndef EIGEN_CONVERSIONS_H_
#define EIGEN_CONVERSIONS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace eigen_conversions
{

/// copies eigen quaternion to geometry_msgs/quaternion
template<class Scalar>
  inline void quaternionToMsg(const Eigen::Quaternion<Scalar> & q_in, geometry_msgs::Quaternion & q_out)
  {
    q_out.w = q_in.w();
    q_out.x = q_in.x();
    q_out.y = q_in.y();
    q_out.z = q_in.z();
  }

/// copies eigen quaternion to geometry_msgs/quaternion
template<class Scalar>
  inline geometry_msgs::Quaternion quaternionToMsg(const Eigen::Quaternion<Scalar> & q_in)
  {
    geometry_msgs::Quaternion q_out;
    quaternionToMsg(q_in, q_out);
    return q_out;
  }

/// copies an eigen 3d vector to a 3d Point struct. point has to have members x,y,z!
template<class Derived, class Point>
  inline void vector3dToPoint(const Eigen::MatrixBase<Derived> & vec, Point & point)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    point.x = vec[0];
    point.y = vec[1];
    point.z = vec[2];
  }

/// copies an eigen 3d vector to a 3d Point struct. point has to have members x,y,z!
template<class Derived, class Point>
  inline Point vector3dToPoint(const Eigen::MatrixBase<Derived> & vec)
  {
    Point point;
    vector3dToPoint(vec, point);
    return point;
  }

}
;

#endif /* EIGEN_CONVERSIONS_H_ */
