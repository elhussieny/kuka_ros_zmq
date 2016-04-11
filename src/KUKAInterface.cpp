#include <iostream>
#include "kuka_ros_zmq/KUKAInterface.h"

KUKAInterface::KUKAInterface(ros::NodeHandle nh_,zmq::context_t & context){
	std::string input_pose_topic = "/kuka_interface/kuka_pose"; // rostopic that will receive the ROS pose

	this->kuka_sub = nh_.subscribe(input_pose_topic,1,&KUKAInterface::kukaGoalCallback,this);

	this->publisher=new zmq::socket_t(context,ZMQ_PUB);
	publisher->bind("tcp://*:5555");

	this->subscriber=new zmq::socket_t(context,ZMQ_SUB);
	subscriber->bind("tcp://*:5558");
	subscriber->setsockopt(ZMQ_SUBSCRIBE,"", 0);


	for(int i=0;i<JOINTSNO;i++) // initialize publishers for Joints
	{
		std::sprintf(this->jntPubName, "/Kuka_q%d",i+1);
		this->kuka_pub_joint[i]=nh_.advertise<std_msgs::Float64>(jntPubName, 1);

	}

}
/*------------------------------------------------------------------------------------*/
KUKAInterface::~KUKAInterface(){
this->destroy();
}
/*------------------------------------------------------------------------------------*/

void KUKAInterface::kukaGoalCallback(const geometry_msgs::PosePtr& intendedPose)
{

										flatbuffers::FlatBufferBuilder fbb;
	     					        	ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);

	     					        	Vector3 pos(intendedPose->position.x,intendedPose->position.y,intendedPose->position.z);
	     					        	Quaternion q(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					      //TODO Need to select between Qaut and ABC
	     					        	Eigen::Quaterniond Rq = Eigen::Quaterniond(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					          	Eigen::Vector3d eulerAngles=Rq.toRotationMatrix().eulerAngles(2,1,0); // ZYX convention
	     					           	RPY rot(eulerAngles[0],eulerAngles[1],eulerAngles[2]); //
	     					        	builder.add_position(&pos);
	     			 		        	builder.add_rotation(&rot);
	     			 		        	builder.add_orientation(&q);
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

void KUKAInterface::readJoints(void){
try{
subscriber->recv(&jointsReply,ZMQ_NOBLOCK);
//if(jointsReply.size()>0)std::cout << "Received Data Size: "  <<jointsReply.size()/*repmsg->angleValue()*/<<std::endl;
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
//std::cout <<"Robot Name:"<<"["<<nameRobot->c_str()<<"]"<<std::endl;
for(int i=0;i<=6;i++){
 //    std::cout <<"Joints:"<<"["<<i<<"]"<<"="<<jointsVVV->Get(i)*180/PI<<" deg"<<std::endl;
     jointMsg.data=jointsVVV->Get(i);
     kuka_pub_joint[i].publish(jointMsg);
}
 //    std::cout << "-------------------- " <<std::endl;

}
/*------------------------------------------------------------------------------------*/

}
void KUKAInterface::destroy(){std::cout<<std::endl<<"Exiting.."<<std::endl;publisher->close();subscriber->close();}


