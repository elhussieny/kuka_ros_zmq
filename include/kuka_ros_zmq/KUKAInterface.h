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
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <math.h>
#include <zmq.hpp>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include "kuka_ros_zmq/kukaCommands_generated.h"
#include "kuka_ros_zmq/kukaMonitor_generated.h"
#include "kuka_ros_zmq/KUKAcontrolConfig.h"
#include "kuka_ros_zmq/KUKACartcontrolConfig.h"

#include <boost/thread.hpp>

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
	ros::Subscriber kuka_joints_sub;								//subscriber for internal ROS message (Desired Joints)
	ros::Subscriber kuka_calib_command;
	ros::Publisher kuka_pub_joint[JOINTSNO];                               //publisher for internal ROS message (Current Joints)
	ros::Publisher kuka_pose_pub;
	zmq::socket_t* publisher;								//zmq publisher for kuka
	zmq::socket_t* subscriber;								//zmq subscriber from kuka
	std::vector<double>jointsV; 							// holds the joints received
	char jntPubName [10];
	std_msgs::Float64 jointMsg;
	zmq::message_t jointsReply;
	ros::Timer* timer;
	geometry_msgs::Pose KUKApose;
	double kukaJoints[JOINTSNO];
	// DH Parameters for the KUKA. Refer to Descriptions.png
	double D1 = 0.36;
	double D3 = 0.42;
	double D5 = 0.40;
	double D7 = 0.126;
	const double JL[JOINTSNO] = { 169.0*PI / 180.0,
	119.0*PI / 180.0,
	169.0*PI / 180.0,
	119.0*PI / 180.0,
	169.0*PI / 180.0,
	119.0*PI / 180.0,
	174.0*PI / 180.0 };

public:
	KUKAInterface(ros::NodeHandle &nh_,zmq::context_t & context);
	~KUKAInterface();
	void kukaGoalCallback(const geometry_msgs::PosePtr& intendedPose);
	void kukaJointsCallback(const sensor_msgs::JointStatePtr& intendedJoints);
	void kukaCalibrate(const std_msgs::StringPtr& command);
	void readJoints();
	bool getInverseKienamatics(const geometry_msgs::PosePtr& desiredPose, double* kJ);


};
#endif /* KUKAINTERFACE_H_ */
