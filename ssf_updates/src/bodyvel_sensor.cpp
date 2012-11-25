/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "bodyvel_sensor.h"
#include <ssf_core/eigen_utils.h>

#define N_MEAS 4 // measurement size
BodyVelSensorHandler::BodyVelSensorHandler(ssf_core::Measurements* meas,const ros::NodeHandle & priv_nh) :
  MeasurementHandler(meas)
{
	priv_nh.param("measurement_world_sensor", measurement_world_sensor_, true);
	priv_nh.param("use_fixed_covariance", use_fixed_covariance_, true);

	ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
	ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

	ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
	ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");

	subscribe(priv_nh);
}

void BodyVelSensorHandler::subscribe(ros::NodeHandle priv_nh)
{
  subMeasurement_ = priv_nh.subscribe("bodyvel_measurement", 1, &BodyVelSensorHandler::measurementCallback, this);

  measurements->ssf_core_.registerCallback(&BodyVelSensorHandler::noiseConfig, this);

  priv_nh.param("meas_noise1", n_zbv_, 0.01);	// default body vel noise is for iof
}

void BodyVelSensorHandler::noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
  //	if(level & ssf_core::SSF_Core_MISC)
  //	{
  this->n_zbv_ = config.meas_noise1;
  //	}
}

void BodyVelSensorHandler::measurementCallback(const sensor_msgs::ImageConstPtr & img)
{
	//	ROS_INFO_STREAM("measurement received \n"
	//					<< "type is: " << typeid(msg).name());

	// init variables
	ssf_core::State state_old;
	ros::Time time_old = img->header.stamp;
	Eigen::Matrix<double, N_MEAS, N_STATE> H_old;
	Eigen::Matrix<double, N_MEAS, 1> r_old;
	Eigen::Matrix<double, N_MEAS, N_MEAS> R;

	H_old.setZero();
	R.setZero();

	unsigned char idx = measurements->ssf_core_.getClosestState(&state_old, time_old);
	if (state_old.time_ == -1)
	return; // // early abort // //

	// get measurements
	double vel[3] = {0,0,0};
	if(!inertialOF.imageCallback(img, vel, state_old.q_int_*state_old.q_ci_))
		return; // // early abort // //

	z_bv_ = Eigen::Matrix<double, 3, 1>(vel[0], vel[1], vel[2]);

	// take fix covariance
	const double s_zbv = n_zbv_ * n_zbv_;
	R = (Eigen::Matrix<double, N_MEAS, 1>() << s_zbv, s_zbv, s_zbv, 1e-6).finished().asDiagonal();

	// feedback for init case
	measurements->p_vc_ = z_bv_;

	// get rotation matrices
	Eigen::Matrix<double, 3, 3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix();
	Eigen::Matrix<double, 3, 3> C_q = state_old.q_.conjugate().toRotationMatrix();
	Eigen::Matrix<double, 3, 3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix();

	// preprocess for elements in H matrix
	double scale=state_old.L_;
	Eigen::Matrix<double,3,1> ew = state_old.w_m_-state_old.b_w_;
	Eigen::Matrix<double,3,3> vec1_sk = skew(C_q*state_old.v_*scale);
	Eigen::Matrix<double,3,3> vec2_sk = skew( C_ci*ew.cross(state_old.p_ci_)*scale);
	Eigen::Matrix<double,3,3> pic_sk = skew(C_ci*C_q*state_old.v_*scale);
	Eigen::Matrix<double,3,3> w_sk = skew(ew);

	// construct H matrix using H-blockx :-)
	H_old.block(0,3,3,3) = C_ci*C_q*scale;
	H_old.block(0,6,3,3) = C_ci*vec1_sk;
	H_old.block(0,15,3,1) = C_ci*C_q*state_old.v_+C_ci*ew.cross(state_old.p_ci_);
	H_old.block(0,19,3,3) = pic_sk+vec2_sk;
	H_old.block(0,22,3,3) = C_ci*w_sk*scale;
	H_old(3,8) = 1.0;	// accept yaw drift

	// construct residuals
	r_old.block(0,0,3,1) = z_bv_ - (C_ci*C_q*state_old.v_+C_ci*ew.cross(state_old.p_ci_))*scale;
	Eigen::Quaternion<double> q_err = state_old.q_;
	r_old(3, 0) = 0;

  // call update step in core class
  measurements->ssf_core_.applyMeasurement(idx, H_old, r_old, R);
}
