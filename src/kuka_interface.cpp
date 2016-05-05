#include <iostream>
#include "kuka_ros_zmq/KUKAInterface.h"
void GUICallback(kuka_ros_zmq::KUKAcontrolConfig &config, uint32_t level);
void GUICallbackPose(kuka_ros_zmq::KUKACartcontrolConfig &config, uint32_t level);
ros::Publisher kukaDesired2;
bool FIRSTRUN = true;
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "kuka_control");
   zmq::context_t context (1);
   ros::NodeHandle nh_;
   ros::Rate rate(10); //Rate of Control Loop

   std::cout<<"Socket Started. Waiting DesPose message..."<<std::endl;
   KUKAInterface kukaObject(nh_,context);
   kukaDesired2 = nh_.advertise<geometry_msgs::Pose>("/kuka_interface/kuka_pose",1);

//
//     dynamic_reconfigure::Server<kuka_ros_zmq::KUKAcontrolConfig> dr_srv;
//   	 dynamic_reconfigure::Server<kuka_ros_zmq::KUKAcontrolConfig>::CallbackType cb;
//   	 cb  =  boost::bind(&GUICallback, _1, _2);
//   	 dr_srv.setCallback(cb);

     dynamic_reconfigure::Server<kuka_ros_zmq::KUKACartcontrolConfig> dr_srvPose;
     dynamic_reconfigure::Server<kuka_ros_zmq::KUKACartcontrolConfig>::CallbackType cbPose;
     cbPose  =  boost::bind(&GUICallbackPose, _1, _2);
   	dr_srvPose.setCallback(cbPose);


 //  	ros::WallTimer timer  =  nh_.createWallTimer(ros::WallDuration(0.1), &KUKAInterface::readJoints, &kukaObject);
   while(ros::ok())
		   {
kukaObject.readJoints();
ros::spinOnce();
//rate.sleep();
   //    ros::spin();
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
//	ROS_INFO("POSE Request: %f %f %f %f %f %f %f",
//			  config.kuka_x,
//			  config.kuka_y,
//			  config.kuka_z,
//			  config.kuka_qx,
//			  config.kuka_qy,
//			  config.kuka_qz,
//			  config.kuka_qw
//
//	  );
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
