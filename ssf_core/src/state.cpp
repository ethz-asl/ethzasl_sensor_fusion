/*
 * state.cpp
 *
 *  Created on: Jun 20, 2012
 *      Author: acmarkus
 */

#include <ssf_core/state.h>

namespace ssf_core
{

void State::reset(){
  // states varying during propagation
  p_.setZero();
  v_.setZero();
  q_.setIdentity();
  b_w_.setZero();
  b_a_.setZero();

  L_ = 1.0;
  q_wv_.setIdentity();
  q_ci_.setIdentity();
  p_ic_.setZero();

  w_m_.setZero();
  a_m_.setZero();

  q_int_.setIdentity();

  P_.setZero();
  time_ = 0;
}

void State::getPoseCovariance(geometry_msgs::PoseWithCovariance::_covariance_type & cov)
{
  assert(cov.size() == 36);

  for (int i = 0; i < 9; i++)
    cov[i / 3 * 6 + i % 3] = P_(i / 3 * N_STATE + i % 3);

  for (int i = 0; i < 9; i++)
    cov[i / 3 * 6 + (i % 3 + 3)] = P_(i / 3 * N_STATE + (i % 3 + 6));

  for (int i = 0; i < 9; i++)
    cov[(i / 3 + 3) * 6 + i % 3] = P_((i / 3 + 6) * N_STATE + i % 3);

  for (int i = 0; i < 9; i++)
    cov[(i / 3 + 3) * 6 + (i % 3 + 3)] = P_((i / 3 + 6) * N_STATE + (i % 3 + 6));
}

void State::getPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose)
{
  eigen_conversions::vector3dToPoint(p_, pose.pose.pose.position);
  eigen_conversions::quaternionToMsg(q_, pose.pose.pose.orientation);
  getPoseCovariance(pose.pose.covariance);
}

void State::getStateMsg(sensor_fusion_comm::ExtState & state)
{
  eigen_conversions::vector3dToPoint(p_, state.pose.position);
  eigen_conversions::quaternionToMsg(q_, state.pose.orientation);
  eigen_conversions::vector3dToPoint(v_, state.velocity);
}

}; // end namespace ssf_core
