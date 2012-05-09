/*
 * pose_sensor.h
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#ifndef POSE_SENSOR_H
#define POSE_SENSOR_H

#include <ssf_core/measurement.h>

class PoseSensorHandler: public MeasurementHandler
{
	// measurements
	Eigen::Quaternion<double> z_q_;	/// attitude measurement camera seen from world
	Eigen::Matrix<double, 3, 1> z_p_;	/// position measurement camera seen from world
	double n_zp_, n_zq_;	/// position and attitude measurement noise

	ros::Subscriber subMeasurement_;
	void subscribe();
	void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
	void noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level);

public:
	PoseSensorHandler(Measurements* meas):MeasurementHandler(meas){subscribe();}
};

#endif /* POSE_SENSOR_H */
