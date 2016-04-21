#include <iostream>
#include "kuka_ros_zmq/KUKAInterface.h"

KUKAInterface::KUKAInterface(ros::NodeHandle &nh_,zmq::context_t & context){



	std::string input_pose_topic = "/kuka_interface/kuka_pose"; // rostopic that will receive the ROS pose
	std::string output_pose_topic = "/kuka_interface/kuka_rec_pose"; // rostopic that will receive the ROS pose
	std::string input_joints_topic = "/kuka_interface/kuka_joints"; // rostopic that will receive the ROS pose
		std::string output_joints_topic = "/kuka_interface/kuka_rec_joints"; // rostopic that will receive the ROS pose

	this->kuka_sub = nh_.subscribe(input_pose_topic,1,&KUKAInterface::kukaGoalCallback,this);
	this->kuka_joints_sub = nh_.subscribe(input_joints_topic,1,&KUKAInterface::kukaJointsCallback,this);
	this->kuka_calib_command = nh_.subscribe("/kuka_start_claib",1,&KUKAInterface::kukaCalibrate,this);

	this->publisher=new zmq::socket_t(context,ZMQ_PUB);
	publisher->setsockopt(ZMQ_SNDHWM,1); // one message only!
	publisher->bind("tcp://*:5555");

	this->subscriber=new zmq::socket_t(context,ZMQ_SUB);


	subscriber->bind("tcp://*:5558");
	subscriber->setsockopt(ZMQ_SUBSCRIBE,"", 0);
	   // Set up a dynamic reconfigure server.
		int j0,j1,j2,j3,j4,j5,j6;

	   nh_.param("/kuka_control/kuka_j0", j0, int(0));
	   nh_.param("/kuka_control/kuka_j1", j1, int(0));
	   nh_.param("/kuka_control/kuka_j2", j2, int(0));
	   nh_.param("/kuka_control/kuka_j3", j3, int(0));
	   nh_.param("/kuka_control/kuka_j4", j4, int(0));
	   nh_.param("/kuka_control/kuka_j5", j5, int(0));
	   nh_.param("/kuka_control/kuka_j6", j6, int(0));

	   for(int i=0;i<JOINTSNO;i++) // initialize publishers for Joints
	{
		std::sprintf(this->jntPubName, "/Kuka_q%d",i+1);
		this->kuka_pub_joint[i]=nh_.advertise<std_msgs::Float64>(jntPubName, 1);

	}
	   this->kuka_pose_pub=nh_.advertise<geometry_msgs::Pose>(output_pose_topic,1);
	//   this->timer = nh_.createTimer(ros::Duration(0.1), &KUKAInterface::readJoints,&this);
}
/*------------------------------------------------------------------------------------*/
KUKAInterface::~KUKAInterface(){
	std::cout<<std::endl<<"Exiting.."<<std::endl;publisher->close();subscriber->close();
	   std::cout<<"Socket Closed!"<<std::endl;
}
/*------------------------------------------------------------------------------------*/

void KUKAInterface::kukaGoalCallback(const geometry_msgs::PosePtr& intendedPose)
{


										flatbuffers::FlatBufferBuilder fbb;
	     					        	ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);
	     					        	ros_kuka::flatbuffer::Vector3 pos(intendedPose->position.x,intendedPose->position.y,intendedPose->position.z);

	     					        	ros_kuka::flatbuffer::Quaternion q(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					      //TODO Need to select between Qaut and ABC
	     					        	Eigen::Quaterniond Rq = Eigen::Quaterniond(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					          	Eigen::Vector3d eulerAngles=Rq.toRotationMatrix().eulerAngles(2,1,0); // ZYX convention of KUKA
	     					           	RPY rot(eulerAngles[0],eulerAngles[1],eulerAngles[2]); //
	     					        	builder.add_position(&pos);
	     			 		        	builder.add_rotation(&rot);
	     			 		        	builder.add_orientation(&q);
	     			 		        	if(pos.x()==0&&pos.y()==0&&pos.z()==0)builder.add_control(ControlMode_BackHome);
	     			 		        //	else builder.add_control(ControlMode_CartesianPTP);
	     			 		      	else builder.add_control(ControlMode_CartesianSS);
	     					        	auto response=builder.Finish();
	     					        	fbb.Finish(response);
//
	     					           zmq::message_t poseRequest (fbb.GetSize());
	     			     memcpy ((void *)poseRequest.data (),fbb.GetBufferPointer(), fbb.GetSize());
//
	     					try{publisher->send(poseRequest,ZMQ_NOBLOCK);}
	     					  catch(const zmq::error_t& ex)
	     					  {printf("number %d \n",ex.num());
	     					 if(ex.num() != EAGAIN) throw;

	     					  }
	     					  printf("Sent Data:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] \n",pos.x(),pos.y(),pos.z(),q.x(),q.y(),q.z(),q.w());

}
/*------------------------------------------------------------------------------------*/
void KUKAInterface::kukaJointsCallback(const sensor_msgs::JointStatePtr& intendedJoints)
{
//TODO
	std::cout<<"Position"<<intendedJoints->position[0]<<std::endl;
	std::cout<<"Velocity"<<intendedJoints->velocity[0]<<std::endl;
//std::vector<double> jointsControl(7,0);
//
//										flatbuffers::FlatBufferBuilder fbb;
//
//										ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);
//
//auto jv=fbb.CreateVector(&jointsControl,7);
//	     			 		        	builder.add_joints(jv);
//	     			 		        	builder.add_control(ControlMode_Joints);
//
//	     					        	auto response=builder.Finish();
//	     					        	fbb.Finish(response);
////
//	     					           zmq::message_t poseRequest (fbb.GetSize());
//	     			     memcpy ((void *)poseRequest.data (),fbb.GetBufferPointer(), fbb.GetSize());
////
//	     					try{publisher->send(poseRequest,ZMQ_NOBLOCK);}
//	     					  catch(const zmq::error_t& ex)
//	     					  {printf("number %d \n",ex.num());
//	     					 if(ex.num() != EAGAIN) throw;
//
//	     					  }
//	     				//	  printf("Sent Data:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] \n",pos.x(),pos.y(),pos.z(),q.x(),q.y(),q.z(),q.w());

}
/*------------------------------------------------------------------------------------*/

void KUKAInterface::readJoints( /*const ros::WallTimerEvent& event*/){
try{
subscriber->recv(&jointsReply,ZMQ_NOBLOCK);
//if(jointsReply.size()>0)std::cout << "Received Data Size: "  <<jointsReply.size()/*repmsg->angleValue()*/<<std::endl;
}
catch(const zmq::error_t& ex)
    {
        // recv() throws ETERM when the zmq context is destroyed,
        //  as when AsyncZmqListener::Stop() is called
if(ex.num() != EAGAIN){printf("Error=%d\n",ex.num());throw;}

    }
if(jointsReply.size()>0){
auto repmsg=kuka_joints::flatbuffer::GetkukaJoints(jointsReply.data());
auto nameRobot=repmsg->robotName();
auto jointsVVV=repmsg->angleValue();
auto robotPos=repmsg->posValue();
auto robotRot=repmsg->rotValue();
////std::cout <<"Robot Name:"<<"["<<nameRobot->c_str()<<"]"<<std::endl;
printf("Position: [%.2f, %.2f, %.2f] \n",robotPos->x(),robotPos->y(),robotPos->z());
printf("Position: [%.2f, %.2f, %.2f, <%.2f>] \n",robotRot->x(),robotRot->y(),robotRot->z(),robotRot->w());
std::cout <<"======================================"<<std::endl;

KUKApose.position.x=robotPos->x();   KUKApose.position.y=robotPos->y();   KUKApose.position.z=robotPos->z();
  KUKApose.orientation.x=robotRot->x();   KUKApose.orientation.y=robotRot->y();   KUKApose.orientation.z=robotRot->z(); KUKApose.orientation.w=robotRot->w();
  this->kuka_pose_pub.publish(KUKApose);


for(int i=0;i<=6;i++){
  // std::cout <<"Joints:"<<"["<<i<<"]"<<"="<<jointsVVV->Get(i)*180/PI<<" deg"<<std::endl;
     jointMsg.data=jointsVVV->Get(i);
     kuka_pub_joint[i].publish(jointMsg);
}
 //    std::cout << "-------------------- " <<std::endl;

}
}
/*------------------------------------------------------------------------------------*/



void KUKAInterface::kukaCalibrate(const std_msgs::StringPtr& command){
double calibPose[10][6]={{-96.97,-575.76,103.03,51.98,1.52,-7.43},
	{-106.08,-513.06,34,35.17,17.75,-64.44},
	{-207.7,-520.39,139.85,62.47,-1.83,-55.15},
	{-170,-569.70,88.28,115.85,41.28,-34.11},
	{-159.82,-437.26,35.81,106.92,27.88,24.86},
	{-244.34,-514.76,50.16,108.21,2.51,-23.56},
	{-244.36,-514.76,50.16,-121.52,16.95,16.80},
	{-158.47,-547.26,113.36,83.50,26.84,1.35},
	{-231.94,-414.8,37.52,92.25,1.15,-9.79},
	{-83.08,-609.83,10.14,95.49,-0.24,22.67}};

	 char fine;
	 std::cout << "Kinect IR Ready ? (y/N) ";
	 std::cin >> fine;
	 printf("Calibration Starts \n");
	 printf("=================== \n");
for(int pose=0;pose<10;pose++)
{

	printf("Point [%d]: \n",pose);
	flatbuffers::FlatBufferBuilder fbb;
 	ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);
 	ros_kuka::flatbuffer::Vector3 pos(calibPose[pose][0],calibPose[pose][1],calibPose[pose][2]);
    	RPY rot(calibPose[pose][3],calibPose[pose][4],calibPose[pose][5]); //
 	builder.add_position(&pos);
  	builder.add_rotation(&rot);
  //	else builder.add_control(ControlMode_CartesianPTP);
	builder.add_control(ControlMode_CartesianPTP);
 	auto response=builder.Finish();
 	fbb.Finish(response);
//
    zmq::message_t poseRequest (fbb.GetSize());
memcpy ((void *)poseRequest.data (),fbb.GetBufferPointer(), fbb.GetSize());
//
try{publisher->send(poseRequest,ZMQ_NOBLOCK);}
catch(const zmq::error_t& ex)
{printf("number %d \n",ex.num());if(ex.num() != EAGAIN) throw;}
printf("Sent Data:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] \n",pos.x(),pos.y(),pos.z(),rot.alpha(),rot.beta(),rot.gamma());

 std::cout << "Daijoubo ? (y/N) ";
 std::cin >> fine;
printf("---------------------\n");

}
printf("Good Luck !!\n");
}




