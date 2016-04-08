/*هيثم الحسيني */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <math.h>
#include <zmq.hpp>
#include <signal.h>
#include "kuka_ros_zmq/kukaState_generated.h"
#include "kuka_ros_zmq/kukaJoints_generated.h"
#define PI 3.1415926

using namespace ros_kuka::flatbuffer;
using namespace kuka_joints::flatbuffer;
/*------------------------------------------------------------------------------------*/
class KUKAInterface{
private:
	ros::Subscriber kuka_sub;								//subscriber for internal ROS message (Desired Pose)
	ros::Subscriber kuka_pub;                               //publisher for internal ROS message (Current Joints)
	zmq::socket_t* publisher;								//zmq publisher for kuka
	zmq::socket_t* subscriber;								//zmq subscriber from kuka
	std::vector<double>jointsV;
public:


	KUKAInterface(ros::NodeHandle nh_,zmq::context_t & context){
		std::string input_pose_topic = "/kuka_interface/kuka_pose"; // rostopic that will receive the ROS pose

		this->kuka_sub = nh_.subscribe(input_pose_topic,1,&KUKAInterface::kukaGoalCallback, this);

		this->publisher=new zmq::socket_t(context,ZMQ_PUB);
		publisher->bind("tcp://*:5555");

		this->subscriber=new zmq::socket_t(context,ZMQ_SUB);
		subscriber->bind("tcp://*:5558");
		subscriber->setsockopt(ZMQ_SUBSCRIBE,"", 0);


	}
	/*------------------------------------------------------------------------------------*/

~KUKAInterface(){
	this->destroy();
}
/*------------------------------------------------------------------------------------*/

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
		     					           zmq::message_t poseRequest (fbb.GetSize());
		     			     memcpy ((void *)poseRequest.data (),fbb.GetBufferPointer(), fbb.GetSize());
//
		     					try{publisher->send(poseRequest);}
		     					  catch(const zmq::error_t& ex)
		     					  {printf("number %d \n",ex.num());
		     					 if(ex.num() != EAGAIN) throw;

		     					  }
		     					  printf("Sent Data:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] \n",pos.x(),pos.y(),pos.z(),rot.alpha(),rot.beta(),rot.gamma());

	}
	/*------------------------------------------------------------------------------------*/

void readJoints(void){
	zmq::message_t jointsReply;

try{
subscriber->recv(&jointsReply,ZMQ_NOBLOCK);
if(jointsReply.size()>0)std::cout << "Received Data Size: "  <<jointsReply.size()/*repmsg->angleValue()*/<<std::endl;
}
catch(const zmq::error_t& ex)
        {
            // recv() throws ETERM when the zmq context is destroyed,
            //  as when AsyncZmqListener::Stop() is called
	if(ex.num() != EAGAIN){printf("Error=%d\n",ex.num());this->destroy();throw;}

        }
if(jointsReply.size()>0){
auto repmsg=kuka_joints::flatbuffer::GetkukaJoints(jointsReply.data());
auto nameRobot=repmsg->robotName();
auto jointsVVV=repmsg->angleValue();
std::cout <<"Robot Name:"<<"["<<nameRobot->c_str()<<"]"<<std::endl;
for(int i=0;i<=6;i++)
	     std::cout <<"Joints:"<<"["<<i<<"]"<<"="<<jointsVVV->Get(i)*180/PI<<" deg"<<std::endl;
         std::cout << "-------------------- " <<std::endl;

}
/*------------------------------------------------------------------------------------*/

}
	void destroy(){std::cout<<std::endl<<"Exiting.."<<std::endl;publisher->close();subscriber->close();}

};
/*-----------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "kuka_control");

   ros::NodeHandle nh_;
   zmq::context_t context (1);
   KUKAInterface kukaObject(nh_,context);
   std::cout<<"Socket Started. Waiting DesPose message..."<<std::endl;

   while(nh_.ok())
		   {
		   ros::spinOnce();

kukaObject.readJoints();

		   }
   kukaObject.destroy();
   std::cout<<"Socket Closed!"<<std::endl;

  return 0;
 }
