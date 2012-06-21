/*
 * measurement.h
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <ssf_core/SSF_Core.h>

namespace ssf_core{

class MeasurementHandler;

class Measurements
{
protected:
	typedef std::vector<MeasurementHandler*> Handlers;
	Handlers handlers;

	ReconfigureServer *reconfServer_;

	void Config(ssf_core::SSF_CoreConfig &config, uint32_t level);
	virtual	void init(double scale) = 0;

public:

	// measurements
	Eigen::Quaternion<double> q_cv_;
	Eigen::Matrix<double, 3, 1> p_vc_;
	Eigen::Matrix<double, 3, 1> v_vc_;
	SSF_Core ssf_core_;

	void addHandler(MeasurementHandler* handler)
	{
		handlers.push_back(handler);
	}
	Measurements();
	virtual ~Measurements();
};


class MeasurementHandler
{
protected:
	Measurements* measurements;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MeasurementHandler(Measurements* meas):measurements(meas){}

	virtual ~MeasurementHandler() {}
};

}; // end namespace

#endif /* MEASUREMENT_H */
