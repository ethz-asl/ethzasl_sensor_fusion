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

class PoseMeasurements: public Measurements
{
public:
	PoseMeasurements()
	{
		addHandler(new PoseSensorHandler(this));
	};

private:
	void init(double scale)
	{
		Eigen::Matrix<double, 3, 1> p,v,b_w,b_a,p_ic,g,w_m,a_m;
		Eigen::Quaternion<double> q, q_ci,q_wv;
		Eigen::Matrix<double,nState_,nState_> P;

		// init values
		g << 0, 0, 9.81;	/// gravity
		b_w << 0,0,0;		/// bias gyroscopes
		b_a << 0,0,0;		/// bias accelerometer

		v << 0,0,0;			/// robot velocity (IMU centered)
		w_m << 0,0,0;		/// initial angular velocity
		a_m =g;				/// initial acceleration

		p_ic = Eigen::Matrix<double,3,1>(0, 0, 0);	/// distance camera-IMU
//		q_ci=Eigen::Quaternion<double>(0, 1, 0, 0);	/// rotation camera-IMU
q_ci=Eigen::Quaternion<double>(0, 1, -1, 0);
		q_ci.normalize();
		q_wv=Eigen::Quaternion<double>(1, 0, 0, 0);	/// vision-world rotation drift
		q_wv.normalize();
		P = Eigen::Matrix<double,nState_,nState_>::Constant(0);	/// error state covariance

		// check if we have already input from the measurement sensor
		if(p_vc_.norm()==0)
			ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
		if((q_cv_.norm()==1) & (q_cv_.w()==1))
			ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

		// calculate initial attitude and position based on sensor measurements
		q = ( q_ci * q_cv_.conjugate() *q_wv).conjugate();
		q.normalize();
		p = q_wv.conjugate().toRotationMatrix()*p_vc_/scale - q.toRotationMatrix()*p_ic ;

		// call initialization in core
		ssf_core_.initialize(p,v,q,b_w,b_a,scale,q_wv,P,w_m,a_m,g,q_ci,p_ic);

		ROS_INFO_STREAM("filter initialized to: \n" <<
						"position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" <<
						"attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]");
	};
};

#endif /* POSE_MEASUREMENTS_H */
