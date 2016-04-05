/*هيثم الحسيني */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <math.h>
#include <zmq.hpp>
#include <signal.h>
#include "kuka_ros_zmq/kukaState_generated.h"
#define PI 3.1415926

using namespace ros_kuka::flatbuffer;
/*------------------------------------------------------------------------------------*/
class KUKAInterface{
private:
	ros::Subscriber kuka_sub;								//subscriber for internal ROS message (Desired Pose)
	ros::Subscriber kuka_pub;                               //publisher for internal ROS message (Current Joints)
	zmq::socket_t* publisher;								//zmq publisher for kuka
	//zmq::socket_t* subscriber;								//zmq subscriber from kuka
public:


	KUKAInterface(ros::NodeHandle nh_,zmq::context_t & context){
		std::string input_pose_topic = "/kuka_interface/kuka_pose"; // rostopic that will receive the ROS pose

		this->kuka_sub = nh_.subscribe(input_pose_topic,1,&KUKAInterface::kukaGoalCallback, this);
		this->publisher=new zmq::socket_t(context,ZMQ_PUB);
		publisher->bind("tcp://*:5555");
	}


	void kukaGoalCallback(const geometry_msgs::PosePtr& intendedPose)
	{

											flatbuffers::FlatBufferBuilder fbb;
		     					        	ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);

		     					        	Vector3 pos(intendedPose->position.x,intendedPose->position.y,intendedPose->position.z);
		     					        	Eigen::Quaterniond Rq = Eigen::Quaterniond(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
		     					          	Eigen::Vector3d eulerAngles=Rq.toRotationMatrix().eulerAngles(2,1,0); // ZYX convention
		     					           	RPY rot(eulerAngles[0],eulerAngles[1],eulerAngles[2]); //
		     					        	builder.add_position(&pos);
		     			 		        	builder.add_rotation(&rot);
		     					        	auto response=builder.Finish();
		     					        	fbb.Finish(response);
//
		     					           zmq::message_t request (fbb.GetSize());
		     			     memcpy ((void *)request.data (),fbb.GetBufferPointer(), fbb.GetSize());
//
		     					try{publisher->send(request);}
		     					  catch(const zmq::error_t& ex)
		     					  {printf("number %d \n",ex.num());
		     					 if(ex.num() != EAGAIN) throw;

		     					  }
		     					  		     					  printf("Sent Data:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] \n",pos.x(),pos.y(),pos.z(),rot.alpha(),rot.beta(),rot.gamma());

	}

	void destroy(){std::cout<<std::endl<<"Exiting.."<<std::endl;publisher->close();}

};
/*-----------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "kuka_control");

   ros::NodeHandle nh_;
   zmq::context_t context (1);
   KUKAInterface kukaObject(nh_,context);
   std::cout<<"Socket Started. Waiting DesPose message..."<<std::endl;
	   while(ros::ok)
		   {ros::spinOnce();



		   }
   kukaObject.destroy();
   std::cout<<"Socket Closed!"<<std::endl;

  return 0;
 }
