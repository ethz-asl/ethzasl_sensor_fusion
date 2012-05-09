/*
 * main.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: Weiss
 */

#ifdef POSE_MEAS
#include "pose_measurements.h"
#endif
#ifdef POSITION_MEAS
#include "position_measurements.h"
#endif

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssf_core");
#ifdef POSE_MEAS
	PoseMeasurements PoseMeas;
	ROS_INFO_STREAM("Filter type: pose_sensor");
#endif

#ifdef POSITION_MEAS
	PositionMeasurements PositionMeas;
	ROS_INFO_STREAM("Filter type: position_sensor");
#endif


	//  print published/subscribed topics
	ros::V_string topics;
	ros::this_node::getSubscribedTopics(topics);
	std::string nodeName = ros::this_node::getName();
	std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
	for(unsigned int i=0; i<topics.size(); i++)
		topicsStr+=("\t\t" + topics.at(i) + "\n");

	topicsStr += "\tadvertised topics:\n";
	ros::this_node::getAdvertisedTopics(topics);
	for(unsigned int i=0; i<topics.size(); i++)
		topicsStr+=("\t\t" + topics.at(i) + "\n");

	ROS_INFO_STREAM(""<< topicsStr);

	ros::spin();

	return 0;
}
