/*
 * position_sensor.h
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#ifndef POSITION_SENSOR_H
#define POSITION_SENSOR_H

#include <ssf_core/measurement.h>
#include <ssf_updates/PositionWithCovarianceStamped.h>


class PositionSensorHandler: public MeasurementHandler
{
	// measurements
	Eigen::Matrix<double, 3, 1> z_p_;
	double n_zp_;

	ros::Subscriber subMeasurement_;
	void subscribe();
    void measurementCallback(const ssf_updates::PositionWithCovarianceStampedConstPtr & msg);
	void noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level);

public:
	PositionSensorHandler(Measurements* meas):MeasurementHandler(meas){subscribe();}
};

#endif /* POSITION_SENSOR_H */
