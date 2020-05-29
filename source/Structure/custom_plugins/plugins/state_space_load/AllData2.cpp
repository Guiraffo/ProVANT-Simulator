/*
* File: AllData2.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor that return all data of the folowing state space:

- x,y,z,roll,pitch,yaw,aR,aL,gammax,gammay,dx,dy,dz,droll,dpitch,dyaw,daR,daL,dgammax,dgammay

*/


#include <AllData2.h>


namespace gazebo
{
	// constructor
	AllData2::AllData2()
	{ 
		
	}
	// destructor
	AllData2::~AllData2()
	{	
		try
		{
			updateTimer.Disconnect(updateConnection);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}
	// initial setup
	void AllData2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // get name of topic to publish data
			NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR",_sdf); // get name of right joint
			NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL",_sdf); // get name of left joint
			NameOfJointLoad_X_ = XMLRead::ReadXMLString("NameOfJointLoad_X",_sdf); // get name of free degree in x of load
			NameOfJointLoad_Y_ = XMLRead::ReadXMLString("NameOfJointLoad_Y",_sdf); // get name of free degree in y of load

			world = _model->GetWorld(); // get world	
			juntaR = _model->GetJoint(NameOfJointR_); // get right joint
			juntaL = _model->GetJoint(NameOfJointL_); // get left joint
			juntaLoadX = _model->GetJoint(NameOfJointLoad_X_); // get free degree in x of load
			juntaLoadY = _model->GetJoint(NameOfJointLoad_Y_); // get free degree in y of load

			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf); // get name of the main body 
			link = _model->GetLink(link_name_); // get main body			
	
			// connection to simulation time
	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&AllData2::Update, this));
			// ROS publisher
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// reset
	void AllData2::Reset()
	{
		try
		{
			updateTimer.Reset();
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// for each time step
	void AllData2::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock); // mutex
			
			ignition::math::Pose3d pose = link->WorldPose(); // get world pose
			ignition::math::Vector3d linear = link->WorldLinearVel(); // get linear velocity pose
			ignition::math::Vector3d angular = link->WorldAngularVel( ); // get angular velocity pose

			simulator_msgs::Sensor newmsg; // time stamp
			newmsg.name = NameOfNode_; // name of node
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(pose.Pos().X()); // x
			newmsg.values.push_back(pose.Pos().Y()); // y
			newmsg.values.push_back(pose.Pos().Z()); // z
			newmsg.values.push_back(pose.Rot().Euler().X()); // roll
			newmsg.values.push_back(pose.Rot().Euler().Y()); // pitch
			newmsg.values.push_back(pose.Rot().Euler().Z()); // yaw
			newmsg.values.push_back(juntaR->Position(0)); // aR
			newmsg.values.push_back(juntaL->Position(0)); // aL
			newmsg.values.push_back(juntaLoadX->Position(0)); // gammax
			newmsg.values.push_back(juntaLoadY->Position(0)); // gammay
			newmsg.values.push_back(linear.X()); // vx
			newmsg.values.push_back(linear.Y()); // vy
			newmsg.values.push_back(linear.Z()); // vz
			newmsg.values.push_back(angular.X()); // wx
			newmsg.values.push_back(angular.Y()); // wy
			newmsg.values.push_back(angular.Z()); // wz
			newmsg.values.push_back(juntaR->GetVelocity(0)); // daR
			newmsg.values.push_back(juntaL->GetVelocity(0)); // daL
			newmsg.values.push_back(juntaLoadX->GetVelocity(0)); // dgammax
			newmsg.values.push_back(juntaLoadY->GetVelocity(0)); // dgammay

			// publisher
			publisher_.publish(newmsg);
					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(AllData2)
}
