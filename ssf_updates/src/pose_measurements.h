/*
 * pose_measurements.h
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#ifndef POSE_MEASUREMENTS_H
#define POSE_MEASUREMENTS_H

#include <ros/ros.h>
#include <ssf_core/measurement.h>
#include "pose_sensor.h"

class PoseMeasurements : public ssf_core::Measurements
{
public:
  PoseMeasurements()
  {
    addHandler(new PoseSensorHandler(this));

    ros::NodeHandle pnh("~");
    pnh.param("init/p_ic/x", p_ic_[0], 0.0);
    pnh.param("init/p_ic/y", p_ic_[1], 0.0);
    pnh.param("init/p_ic/z", p_ic_[2], 0.0);

    pnh.param("init/q_ci/w", q_ci_.w(), 1.0);
    pnh.param("init/q_ci/x", q_ci_.x(), 0.0);
    pnh.param("init/q_ci/y", q_ci_.y(), 0.0);
    pnh.param("init/q_ci/z", q_ci_.z(), 0.0);

    if (q_ci_.norm() != 1.0)
    {
      ROS_WARN_STREAM("initial q_ci" << q_ci_.coeffs().transpose() <<") is not normalized, normalizing it now");
      q_ci_.normalize();
    }
  }

private:

  Eigen::Matrix<double, 3, 1> p_ic_; ///< initial distance camera-IMU
  Eigen::Quaternion<double> q_ci_; ///< initial rotation camera-IMU

  void init(double scale)
  {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m;
    Eigen::Quaternion<double> q, q_wv;
    ssf_core::SSF_Core::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81; // gravity
    b_w << 0, 0, 0; // bias gyroscopes
    b_a << 0, 0, 0; // bias accelerometer

    v << 0, 0, 0; // robot velocity (IMU centered)
    w_m << 0, 0, 0; // initial angular velocity
    a_m = g; // initial acceleration

    q_wv.setIdentity(); // vision-world rotation drift

    P.setZero(); // error state covariance; if zero, a default initialization in ssf_core is used

    // check if we have already input from the measurement sensor
    if (p_vc_.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
    if ((q_cv_.norm() == 1) & (q_cv_.w() == 1))
      ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

    // calculate initial attitude and position based on sensor measurements
    q = (q_ci_ * q_cv_.conjugate() * q_wv).conjugate();
    q.normalize();
    p = q_wv.conjugate().toRotationMatrix() * p_vc_ / scale - q.toRotationMatrix() * p_ic_;

    // call initialization in core
    ssf_core_.initialize(p, v, q, b_w, b_a, scale, q_wv, P, w_m, a_m, g, q_ci_, p_ic_);

    ROS_INFO_STREAM("filter initialized to: \n" <<
        "position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl <<
        "scale:" << scale << std::endl <<
        "attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl <<
        "p_ic: [" << p_ic_[0] << ", " << p_ic_[1] << ", " << p_ic_[2] << std::endl <<
        "q_ci: (w,x,y,z): [" << q_ci_.w() << ", " << q_ci_.x() << ", " << q_ci_.y() << ", " << q_ci_.z() << "]");
  }
};

#endif /* POSE_MEASUREMENTS_H */
