/*
 * eigen_utils.h
 *
 *  Created on: Jun 14, 2012
 *      Author: acmarkus
 */

#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

/// returns the 3D cross product skew symmetric matrix of a given 3D vector
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
  }

/// returns a matrix with angular velocities used for quaternion derivatives/integration with the JPL notation
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param <vec> {3D vector with angular velocities}
 \return {4x4 matrix for multiplication with the quaternion}
 */
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatJPL(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (
        Eigen::Matrix<typename Derived::Scalar, 4, 4>() <<
        0, vec[2], -vec[1], vec[0],
        -vec[2], 0, vec[0], vec[1],
        vec[1], -vec[0], 0, vec[2],
        -vec[0], -vec[1], -vec[2], 0
        ).finished();
  }

/// returns a matrix with angular velocities used for quaternion derivatives/integration with the Hamilton notation
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param <vec> {3D vector with angular velocities}
 \return {4x4 matrix for multiplication with the quaternion}
 */
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatHamilton(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (
        Eigen::Matrix<typename Derived::Scalar, 4, 4>() <<
        0, -vec[2], vec[1], vec[0],
        vec[2], 0, -vec[0], vec[1],
        -vec[1], vec[0], 0, vec[2],
        -vec[0], -vec[1], -vec[2], 0
        ).finished();
  }

/// computes a quaternion from the 3-element small angle approximation theta
template<class Derived>
  Eigen::Quaternion<typename Derived::Scalar> quaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta)
  {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

    if ( q_squared < 1)
    {
      return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
    }
    else
    {
      const Scalar w = 1.0 / sqrt(1 + q_squared);
      const Scalar f = w*0.5;
      return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
    }
  }

#endif /* EIGEN_UTILS_H_ */
