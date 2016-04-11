/*
 * KUKAInterface.h
 *
 *  Created on: Apr 9, 2016
 *      Author: haitham
 */

#ifndef KUKAINTERFACE_H_
#define KUKAINTERFACE_H_
/*هيثم الحسيني */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <math.h>
#include <zmq.hpp>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include "kuka_ros_zmq/kukaState_generated.h"
#include "kuka_ros_zmq/kukaJoints_generated.h"
#include "kuka_ros_zmq/KUKAcontrolConfig.h"
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include"kuka_ros_zmq/KUKADefinition.h"


using namespace boost;
using namespace ros_kuka::flatbuffer;
using namespace kuka_joints::flatbuffer;
class KUKAInterface{
private:
	ros::Subscriber kuka_sub;								//subscriber for internal ROS message (Desired Pose)
	ros::Publisher kuka_pub_joint[JOINTSNO];                               //publisher for internal ROS message (Current Joints)
	zmq::socket_t* publisher;								//zmq publisher for kuka
	zmq::socket_t* subscriber;								//zmq subscriber from kuka
	std::vector<double>jointsV; 							// holds the joints received
	char jntPubName [8];
	std_msgs::Float64 jointMsg;
	zmq::message_t jointsReply;

public:
	KUKAInterface(ros::NodeHandle nh_,zmq::context_t & context);
	~KUKAInterface();
	void kukaGoalCallback(const geometry_msgs::PosePtr& intendedPose);
	void readJoints(void);
	void destroy();
	void GUICallback(kuka_ros_zmq::KUKAcontrolConfig &config, uint32_t level);

};
#endif /* KUKAINTERFACE_H_ */
