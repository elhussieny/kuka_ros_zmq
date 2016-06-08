#include <iostream>
#include "kuka_ros_zmq/KUKAInterface.h"
void GUICallback(kuka_ros_zmq::KUKAcontrolConfig &config, uint32_t level);
void GUICallbackPose(kuka_ros_zmq::KUKACartcontrolConfig &config, uint32_t level);
ros::Publisher kukaDesired2;
bool FIRSTRUN = true;
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "kuka_control");
   ros::NodeHandle nh_;

   zmq::context_t context (1);
   ros::Rate rate(10); //Rate of Control Loop
   std::cout<<"Socket Started. Waiting DesPose aeee message..."<<std::endl;
   KUKAInterface kukaObject(nh_, context);
  kukaDesired2 = nh_.advertise<geometry_msgs::Pose>("/kuka_interface/kuka_pose",1);
// For rqt_reconfigure
     dynamic_reconfigure::Server<kuka_ros_zmq::KUKACartcontrolConfig> dr_srvPose;
     dynamic_reconfigure::Server<kuka_ros_zmq::KUKACartcontrolConfig>::CallbackType cbPose;
     cbPose  =  boost::bind(&GUICallbackPose, _1, _2);
   	dr_srvPose.setCallback(cbPose);


   while(ros::ok())
		   {
kukaObject.readJoints();
ros::spinOnce();
		   }


  return 0;
 }

void GUICallback(kuka_ros_zmq::KUKAcontrolConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %d",
            config.kuka_j0,
            config.mode);
}

void GUICallbackPose(kuka_ros_zmq::KUKACartcontrolConfig &config, uint32_t level) {
	if(FIRSTRUN==false){ // not at begining!
	geometry_msgs::Pose testPose;
	testPose.position.x  =  config.kuka_x;
	testPose.position.y  =  config.kuka_y;
	testPose.position.z  =  config.kuka_z;

	testPose.orientation.x  =  config.kuka_qx;
	testPose.orientation.y  =  config.kuka_qy;
	testPose.orientation.z  =  config.kuka_qz;
	testPose.orientation.w = config.kuka_qw;
	kukaDesired2.publish(testPose);

	}


	FIRSTRUN=false;



}
