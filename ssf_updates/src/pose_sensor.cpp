/*
 * pose_sensor.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#include "pose_sensor.h"
#include <ssf_core/eigen_utils.h>

#define nMeas_ 7 // measurement size

void PoseSensorHandler::subscribe(){

	ros::NodeHandle nh("ssf_core");
	subMeasurement_ = nh.subscribe("pose_measurement", 1, &PoseSensorHandler::measurementCallback, this);

	measurements->ssf_core_.registerCallback(&PoseSensorHandler::noiseConfig, this);

	nh.param("meas_noise1",n_zp_,0.01);
	nh.param("meas_noise2",n_zq_,0.02);
}

void PoseSensorHandler::noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level){
	//	if(level & ssf_core::SSF_Core_MISC)
	//	{
		this->n_zp_ = config.meas_noise1;
		this->n_zq_ = config.meas_noise2;
	//	}
}


void PoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
//	ROS_INFO_STREAM("measurement received \n"
//					<< "type is: " << typeid(msg).name());

	// init variables
	State state_old;
	ros::Time time_old = msg->header.stamp;
	Eigen::Matrix<double,nMeas_,nState_>H_old;
	Eigen::Matrix<double, nMeas_, 1> r_old(nMeas_);
	Eigen::Matrix<double,nMeas_,nMeas_> R;

	H_old.setZero();
	R.setZero();

	// get measurements
	z_p_ = Eigen::Matrix<double,3,1>(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	z_q_ = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

	/*************************************************************************************/
	// use this if your pose sensor is ethzasl_ptam (www.ros.org/wiki/ethzasl_ptam)
	// ethzasl_ptam publishes the camera pose as the world seen from the camera
	z_p_ = -z_q_.toRotationMatrix()*z_p_;
	z_q_ = z_q_.conjugate();
	/*************************************************************************************/

	// take covariance from sensor
	R.block<6,6>(0,0) = Eigen::Matrix<double,6,6>(&msg->pose.covariance[0]);
	R.block<3,3>(0,0) = z_q_.toRotationMatrix().transpose()*R.block<3,3>(0,0);
	R.block<3,3>(3,3) = z_q_.toRotationMatrix().transpose()*R.block<3,3>(3,3);
	for(int i=0; i<nMeas_*nMeas_;i++)
		R(i) = fabs(R(i));
	Eigen::Matrix<double,6,1> buffvec = R.block<6,6>(0,0).diagonal();
	R.block<6,6>(0,0) = buffvec.asDiagonal();	// use this if your pose sensor is ethzasl_ptam (www.ros.org/wiki/ethzasl_ptam)
	R(6,6)=1e-6;	// q_vw yaw-measurement noise

	//  alternatively take fix covariance from reconfigure GUI
//	Eigen::Matrix<double,nMeas_,1> buffvec;
//	buffvec  << n_zp_*n_zp_,n_zp_*n_zp_,n_zp_*n_zp_,n_zq_*n_zq_,n_zq_*n_zq_,n_zq_*n_zq_,1e-6; // last element is q_vw yaw-measurement noise
//	R = buffvec.asDiagonal();

	// feedback for init case
	measurements->p_vc_ = z_p_;
	measurements->q_cv_ = z_q_;

	unsigned char idx = measurements->ssf_core_.getClosestState(&state_old,time_old);
	if (state_old.time_ == -1)
		return;	// // early abort // //

	// get rotation matrices
	Eigen::Matrix<double,3,3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix();
	Eigen::Matrix<double,3,3> C_q = state_old.q_.conjugate().toRotationMatrix();
	Eigen::Matrix<double,3,3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix();

	// preprocess for elements in H matrix
	Eigen::Matrix<double,3,1> vecold;
	vecold = (state_old.p_+C_q.transpose()*state_old.p_ic_)*state_old.L_;
	Eigen::Matrix<double,3,3> skewold = skew(vecold);

	Eigen::Matrix<double,3,3> pic_sk = skew(state_old.p_ic_);

	// construct H matrix using H-blockx :-)
	// position:
	H_old.block<3,3>(0,0) = C_wv.transpose()*state_old.L_; // p
	H_old.block<3,3>(0,6) = -C_wv.transpose()*C_q.transpose()*pic_sk*state_old.L_; // q
	H_old.block<3,1>(0,15) = C_wv.transpose()*C_q.transpose()*state_old.p_ic_ + C_wv.transpose()*state_old.p_; // L
	H_old.block<3,3>(0,16) = -C_wv.transpose()*skewold; // q_wv
	H_old.block<3,3>(0,22) = C_wv.transpose()*C_q.transpose()*state_old.L_; //p_ic
	// attitude
	H_old.block<3,3>(3,6) = C_ci; // q
	H_old.block<3,3>(3,16) = C_ci*C_q; // q_wv
	H_old.block<3,3>(3,19) = Eigen::Matrix<double,3,3>::Identity(); //q_ci
	H_old(6,18) = 1.0;	// fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

	// construct residuals
	// position
	r_old.block<3,1>(0,0) = z_p_ - C_wv.transpose()*(state_old.p_ + C_q.transpose()*state_old.p_ic_)*state_old.L_;
	// attitude
	Eigen::Quaternion<double> q_err;
	q_err = (state_old.q_wv_*state_old.q_*state_old.q_ci_).conjugate()*z_q_;
	r_old.block<3,1>(3,0) = q_err.vec()/q_err.w()*2;
	// vision world yaw drift
	q_err = state_old.q_wv_;
	r_old(6,0) = -2*(q_err.w()*q_err.z()+q_err.x()*q_err.y())/(1-2*(q_err.y()*q_err.y()+q_err.z()*q_err.z()));

	// call update step in core class
	measurements->ssf_core_.applyMeasurement(idx,H_old,r_old,R);
}
