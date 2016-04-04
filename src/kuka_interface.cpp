#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <math.h>
#include <zmq.hpp>
#include "kuka_ros_zmq/kukaState_generated.h"
using namespace ros_kuka::flatbuffer;
#define PI 3.1415926
/*-----------------------------------------------------------------------------------------------------*/
void KukaGoalCallback(const geometry_msgs::PosePtr& intendedPose)
{
	flatbuffers::FlatBufferBuilder fbb;
	     					        	ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);

	     					        	Vector3 pos(intendedPose->position.x,intendedPose->position.y,intendedPose->position.z);
	     					        	Eigen::Quaterniond Rq = Eigen::Quaterniond(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					          	Eigen::Vector3d eulerAngles=Rq.toRotationMatrix().eulerAngles(2,1,0); // ZXZ
	     					           	RPY rot(eulerAngles[0],eulerAngles[1],eulerAngles[2]); // spped testing
	     					        	builder.add_position(&pos);
	     			 		        	builder.add_rotation(&rot);
	     					        	auto response=builder.Finish();
	     					        	fbb.Finish(response);
	     					        	//  Prepare our context and socket
	     					        	zmq::context_t context (1);
	     					        	zmq::socket_t publisher (context, ZMQ_PUB);
	     					        	publisher.bind("tcp://*:5555");//tcp://127.0.0.1:9998
	     					            zmq::message_t request (fbb.GetSize());
	     			        memcpy ((void *)request.data (),fbb.GetBufferPointer(), fbb.GetSize());

	     					try{publisher.send(request);}
	     					  catch(const zmq::error_t& ex)
	     					  {
	     						  printf("numer %d \n",ex.num());
					             throw;
	     	    		        }


}

/*-----------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "kuka_control");
   ros::NodeHandle nh;
   ros::Rate rate(20); // 30hz
   ros::Subscriber sub = nh.subscribe("/kuka_interface/kuka_pose",1, KukaGoalCallback);

  return 0;
 }
