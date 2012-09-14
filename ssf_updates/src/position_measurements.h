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

#ifndef POSITION_MEASUREMENTS_H
#define POSITION_MEASUREMENTS_H

#include <ros/ros.h>
#include <ssf_core/measurement.h>
#include "position_sensor.h"

class PositionMeasurements: public ssf_core::Measurements
{
public:
	PositionMeasurements()
	{
		addHandler(new PositionSensorHandler(this));
	};

private:
	void init(double scale)
	{
		Eigen::Matrix<double, 3, 1> p,v,b_w,b_a,p_ci,g,w_m,a_m;
		Eigen::Quaternion<double> q, q_ci,q_wv;
		Eigen::Matrix<double,N_STATE,N_STATE> P;

		// init values
		g << 0, 0, 9.81;	/// gravity
		b_w << 0,0,0;		/// bias gyroscopes
		b_a << 0,0,0;		/// bias accelerometer

		v << 0,0,0;			/// robot velocity (IMU centered)
		w_m << 0,0,0;		/// initial angular velocity
		a_m =g;				/// initial acceleration

		p_ci = Eigen::Matrix<double,3,1>(0, 0, 0);	/// distance camera-IMU
		q_ci=Eigen::Quaternion<double>(1, 0, 0, 0);	/// rotation camera-IMU
		q_ci.normalize();
		q_wv=Eigen::Quaternion<double>(1, 0, 0, 0);	/// vision-world rotation drift
		q_wv.normalize();
		P = Eigen::Matrix<double,N_STATE,N_STATE>::Constant(0);	/// error state covariance

		// check if we have already input from the measurement sensor
		if(p_vc_.norm()==0)
			ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
		if((q_cv_.norm()==1) & (q_cv_.w()==1))
			ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

		// calculate initial attitude and position based on sensor measurements
		q = ( q_ci /**q_cv_*/ *q_wv).conjugate(); // we do not have a vision frame here...
		q.normalize();
		p = q_wv.conjugate().toRotationMatrix()*p_vc_/scale - q.toRotationMatrix()*p_ci;

		// call initialization in core
		ssf_core_.initialize(p,v,q,b_w,b_a,scale,q_wv,P,w_m,a_m,g,q_ci,p_ci);

		ROS_INFO_STREAM("filter initialized to: \n" <<
						"position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" <<
						"attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]");
	};
};

#endif /* POSITION_MEASUREMENTS_H */

