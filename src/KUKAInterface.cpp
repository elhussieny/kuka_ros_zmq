#include <iostream>
#include "kuka_ros_zmq/KUKAInterface.h"

KUKAInterface::KUKAInterface(ros::NodeHandle &nh_, zmq::context_t & context){
	std::string input_pose_topic = "/kuka_interface/kuka_pose"; // rostopic that will receive the ROS pose
	std::string output_pose_topic = "/kuka_interface/kuka_rec_pose"; // rostopic that will receive the ROS pose
	std::string input_joints_topic = "/kuka_interface/kuka_joints"; // rostopic that will receive the ROS pose
		std::string output_joints_topic = "/kuka_interface/kuka_rec_joints"; // rostopic that will receive the ROS pose

	this->kuka_sub = nh_.subscribe(input_pose_topic,1,&KUKAInterface::kukaGoalCallback,this);

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
std::cout<<"Choose the control mode ? [0:Direct   1:Shared    2:Autonomous]"<<std::endl;
std::cin>>this->control_mode;
std::cout<<"control mode: "<<this->control_mode<<std::endl;
}
/*------------------------------------------------------------------------------------*/
KUKAInterface::~KUKAInterface(){
	std::cout<<std::endl<<"Exiting.."<<std::endl;publisher->close();subscriber->close();
	   std::cout<<"Socket Closed!"<<std::endl;
}
/*------------------------------------------------------------------------------------*/

void KUKAInterface::kukaGoalCallback(const geometry_msgs::PosePtr& intendedPose)
{

	// For bounded reigon (For Safety)
/*	intendedPose->position.x = BOUND(intendedPose->position.x,KUKA_X_MIN,KUKA_X_MAX);
	intendedPose->position.y = BOUND(intendedPose->position.y,KUKA_Y_MIN,KUKA_Y_MAX);
	intendedPose->position.z = BOUND(intendedPose->position.z,KUKA_Z_MIN,KUKA_Z_MAX);
*/
	printf("Pose:[%.3f,%.3f,%.3f] \n",intendedPose->position.x,intendedPose->position.y,intendedPose->position.z);

	if(!getInverseKienamatics(intendedPose,kukaJoints))
	{
		printf("Unavailable IK solution! \n");
		return;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

										flatbuffers::FlatBufferBuilder fbb;
										auto jointsIndex=fbb.CreateVector(kukaJoints,JOINTSNO);

	     					        	ros_kuka::flatbuffer::kukaDesPoseBuilder builder(fbb);

	     					        	ros_kuka::flatbuffer::Vector3 pos(BOUND(intendedPose->position.x*1000,KUKA_X_MIN,KUKA_X_MAX),
	     					        			BOUND(intendedPose->position.y*1000,KUKA_Y_MIN,KUKA_Y_MAX),
	     					        			BOUND(intendedPose->position.z*1000,KUKA_Z_MIN,KUKA_Z_MAX));


	     					        	ros_kuka::flatbuffer::Quaternion q(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					      //TODO Need to select between Qaut and ABC
	     					        	Eigen::Quaterniond Rq = Eigen::Quaterniond(intendedPose->orientation.x,intendedPose->orientation.y,intendedPose->orientation.z,intendedPose->orientation.w);
	     					          	Eigen::Vector3d eulerAngles=Rq.toRotationMatrix().eulerAngles(2,1,0); // ZYX convention of KUKA
	     					           	RPY rot(eulerAngles[0],eulerAngles[1],eulerAngles[2]); //
	     					        //	builder.add_position(&pos);
	     			 		        //	builder.add_rotation(&rot);
	     			 		        //	builder.add_orientation(&q);
	     			 		        	builder.add_joints(jointsIndex); // Now we solve IK and sendjoint values

	     			 		       // 	if(pos.x()==0&&pos.y()==0&&pos.z()==0)builder.add_control(ControlMode_BackHome);
	     			 		        //	else builder.add_control(ControlMode_CartesianPTP);
	     			 		      	//else builder.add_control(ControlMode_CartesianSS);
	     			 		        	builder.add_control(ControlMode_Joints);

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
	     	//				printf("Sent Joints:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] \n",kukaJoints[0]*180/PI,
	     	//						kukaJoints[1]*180/PI,kukaJoints[2]*180/PI,kukaJoints[3]*180/PI,kukaJoints[4]*180/PI,
	     		//					kukaJoints[5]*180/PI,kukaJoints[6]*180/PI);

}
/*------------------------------------------------------------------------------------*/
void KUKAInterface::readJoints( /*const ros::WallTimerEvent& event*/){

	//printf("Error");
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
//std::cout <<"Robot Name:"<<"["<<nameRobot->c_str()<<"]"<<std::endl;
/*
printf("Position: [%.2f, %.2f, %.2f] \n",robotPos->x(),robotPos->y(),robotPos->z());
printf("Oriention: [%.2f, %.2f, %.2f, <%.2f>] \n",robotRot->x(),robotRot->y(),robotRot->z(),robotRot->w());
std::cout <<"======================================"<<std::endl;
*/

KUKApose.position.x=robotPos->x()/1000;   KUKApose.position.y=robotPos->y()/1000;   KUKApose.position.z=robotPos->z()/1000;
  KUKApose.orientation.x=robotRot->x();   KUKApose.orientation.y=robotRot->y();   KUKApose.orientation.z=robotRot->z(); KUKApose.orientation.w=robotRot->w();
  this->kuka_pose_pub.publish(KUKApose);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

for(int i=0;i<=6;i++){
  // std::cout <<"Joints:"<<"["<<i<<"]"<<"="<<jointsVVV->Get(i)*180/PI<<" deg"<<std::endl;
     jointMsg.data=jointsVVV->Get(i);
     kuka_pub_joint[i].publish(jointMsg);
}
 //    std::cout << "-------------------- " <<std::endl;

}
}
/*------------------------------------------------------------------------------------*/
bool KUKAInterface::getInverseKienamatics(const geometry_msgs::PosePtr& desiredPose, double* kJ)
{
    bool output = true;

	// Auxiliary variables
		double mod_pW, mod_pWxy, c2, s2, c3, s3;
			Eigen::Vector3d p(desiredPose->position.x,desiredPose->position.y, desiredPose->position.z-D1);
			Eigen::Quaterniond R = Eigen::Quaterniond(0,0,-0.70711,-0.70711);


            Eigen::Vector3d pW = p-(D7 *R.toRotationMatrix().col(2));
				// Calculate wrist position
			kJ[0] = atan2(pW[1], pW[0]);
						mod_pW = pow(pW.norm(),2); // Pwx^2+Pwy^2+Pwz^2

					c3 = (mod_pW - D3*D3 - D5*D5)/(2*D3*D5);
					// If c3>1, there is no solution for IKT
					if (c3>1){/*printf("NOT --- REACHABLE! c=%.3f \n",c3);*/output= false;
					for(int j=0;j<7;j++) kJ[j]=0;

					return output;
					}
					//
						s3 = -sqrt(1 - c3*c3);
						kJ[3] = atan2(s3, c3) + PI / 2;
						//
						//    					// We do not use the extra dof for getting the inverse kinematics
						kJ[2] = 0.0;
						//
						mod_pWxy = sqrt(pW[0] * pW[0] + pW[1] * pW[1]);
						s2 = ((D3 + D5*c3)*pW[2] - D5*s3*mod_pWxy) / mod_pW;
						c2 = ((D3 + D5*c3)*mod_pWxy + D5*s3*pW[2]) / mod_pW;
						kJ[1] = atan2(s2, c2);
						//
						//    					// Calculate orientation (angles of the wrist joints)
						Eigen::Matrix3d T01; T01 << cos(kJ[0]), 0.0, sin(kJ[0]), sin(kJ[0]), 0.0, -cos(kJ[0]), 0.0, 1.0, 0.0;
						Eigen::Matrix3d T12; T12 << cos(kJ[1]), -sin(kJ[1]), 0.0, sin(kJ[1]), cos(kJ[1]), 0.0, 0.0, 0.0, 1.0;
						Eigen::Matrix3d T23; T23 << cos(kJ[3]), 0.0, sin(kJ[3]), sin(kJ[3]), 0.0, -cos(kJ[3]), 0.0, 1.0, 0.0;
						//
						Eigen::Matrix3d pose03 = T01*T12*T23;
						Eigen::Matrix3d  pose36 = pose03.inverse() * (R.toRotationMatrix());
						//
						kJ[4] = atan2(pose36(1, 2), pose36(0, 2));
						kJ[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
						kJ[6] = atan2(pose36(2, 1), -pose36(2, 0));
						//
						//    					//Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
						kJ[1] < -PI / 2 ? kJ[1] += 3 * PI / 2 : kJ[1] -= PI / 2;
						kJ[3] < -PI / 2 ? kJ[3] += 3 * PI / 2 : kJ[3] -= PI / 2;
						kJ[6] <     0 ? kJ[6] += PI : kJ[6] -= PI;

						kJ[1] = -kJ[1]; //Correcting for the RobotRotation
						kJ[5] = -kJ[5]; //Correcting for the RobotRotation

						//
						for (int i = 0; i < 7; i++){
							if (fabs(kJ[i]) > JL[i]){
								output = false;
								break;
								kJ[i] > 0 ? kJ[i] = JL[i] : kJ[i] = -JL[i];
								printf("Warning!!! IK gives values out of bounds for joint %d \n", i);
							}
							//
							printf("Joint [%d]: %.3f \n", i, kJ[i] * 180 / PI);
						}
						//
						//
						return output;


}



