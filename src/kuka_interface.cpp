#include <iostream>
#include "kuka_ros_zmq/KUKAInterface.h"
void GUICallback(kuka_ros_zmq::KUKAcontrolConfig &config, uint32_t level);
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "kuka_control");
   ros::NodeHandle nh_;
   zmq::context_t context (1);

   KUKAInterface kukaObject(nh_,context);
   ros::Rate rate(300); //30Hz

   std::cout<<"Socket Started. Waiting DesPose message..."<<std::endl;
   // Set up a dynamic reconfigure server.
	int j0,j1,j2,j3,j4,j5,j6;

   nh_.param("/kuka_control/kuka_j0", j0, int(0));
   nh_.param("/kuka_control/kuka_j1", j1, int(0));
   nh_.param("/kuka_control/kuka_j2", j2, int(0));
   nh_.param("/kuka_control/kuka_j3", j3, int(0));
   nh_.param("/kuka_control/kuka_j4", j4, int(0));
   nh_.param("/kuka_control/kuka_j5", j5, int(0));
   nh_.param("/kuka_control/kuka_j6", j6, int(0));

     dynamic_reconfigure::Server<kuka_ros_zmq::KUKAcontrolConfig> dr_srv;
   	 dynamic_reconfigure::Server<kuka_ros_zmq::KUKAcontrolConfig>::CallbackType cb;
   	 cb = boost::bind(&GUICallback, _1, _2);
   	 dr_srv.setCallback(cb);


   while(nh_.ok())
		   {
kukaObject.readJoints();
ros::spinOnce();
rate.sleep();
		   }
  kukaObject.destroy();
   std::cout<<"Socket Closed!"<<std::endl;

  return 0;
 }

void GUICallback(kuka_ros_zmq::KUKAcontrolConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %d",
            config.kuka_j0,
            config.mode);
}

