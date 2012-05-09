/*
 * measurement.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#include <ssf_core/measurement.h>

Measurements::Measurements()
{
	// setup: initial pos, att, of measurement sensor

	p_vc_ = Eigen::Matrix<double, 3, 1>::Constant(0);
	q_cv_ = Eigen::Quaternion<double>(1, 0, 0, 0);

	ssf_core_.registerCallback(&Measurements::Config,this);
}

Measurements::~Measurements()
{
	for (Handlers::iterator it(handlers.begin()); it != handlers.end(); ++it)
		delete *it;

	delete reconfServer_;
	return;
}


void Measurements::Config(ssf_core::SSF_CoreConfig& config, uint32_t level){
	if(level & ssf_core::SSF_Core_INIT_FILTER){
		init(config.scale_init);
		config.init_filter = false;
	}
	else if(level & ssf_core::SSF_Core_SET_HEIGHT){
		if(p_vc_.norm()==0)
		{
			ROS_WARN_STREAM("No measurements received yet to initialize position - using scale factor " << config.scale_init << " for init");
			init(config.scale_init);
		}
		else
		{
			init(p_vc_[2]/config.height);
			ROS_WARN_STREAM("init filter (set scale to: " << p_vc_[2]/config.height << ")");
		}
		config.set_height = false;
	}
}




