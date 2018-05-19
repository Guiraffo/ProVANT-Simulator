/*
* File: AlData.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor that return all data of the folowing state space:

- x,y,z,roll,pitch,yaw,aR,aL,dx,dy,dz,droll,dpitch,dyaw,daR,daL

*/


#include <AllData.h>


namespace gazebo
{
	// constructor
	AllData::AllData()
	{ 
				
	}

	// destructor
	AllData::~AllData()
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
	void AllData::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "entrei";
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // Get name of topic to publish data
			NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR",_sdf); // name of the right joint
			NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL",_sdf); // name of the left joint
			
			world = _model->GetWorld();	// pointer to the world
			juntaR = _model->GetJoint(NameOfJointR_); // pointer to the right joint
			juntaL = _model->GetJoint(NameOfJointL_); // pointer to the left joint

			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf); // name of the main body
			link = _model->GetLink(link_name_); // pointer to the main body			

			// notifying when occurs new step time
	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&AllData::Update, this));
			// ROS publisher
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// reset
	void AllData::Reset()
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

	// new step time
	void AllData::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock); // mutex
			
			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now(); // time stamp
			newmsg.header.frame_id = "1";
			math::Pose pose = link->GetWorldPose();
			newmsg.values.push_back(pose.pos.x); // x
			newmsg.values.push_back(pose.pos.y); // y
			newmsg.values.push_back(pose.pos.z); // z
			newmsg.values.push_back(pose.rot.GetAsEuler( ).x); //roll
			newmsg.values.push_back(pose.rot.GetAsEuler( ).y); // pitch
			newmsg.values.push_back(pose.rot.GetAsEuler( ).z); // yaw
			newmsg.values.push_back(juntaR->GetAngle(0).Radian()); // aR
			newmsg.values.push_back(juntaL->GetAngle(0).Radian()); // aL
			math::Vector3 linear = link->GetWorldLinearVel();
			newmsg.values.push_back(linear.x); // vx
			newmsg.values.push_back(linear.y); // vy
			newmsg.values.push_back(linear.z); //vz
			math::Vector3 angular = link->GetWorldAngularVel( );
			// droll -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(angular.x*(1) + angular.y*((sin(pose.rot.GetAsEuler( ).x)*sin(pose.rot.GetAsEuler( ).y))/cos(pose.rot.GetAsEuler( ).y)) + angular.z*((cos(pose.rot.GetAsEuler( ).x)*sin(pose.rot.GetAsEuler( ).y))/cos(pose.rot.GetAsEuler( ).y)));
			// dpitch  -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(angular.x*(0) + angular.y*(cos(pose.rot.GetAsEuler( ).x)) + angular.z*(-sin(pose.rot.GetAsEuler( ).x)));
			// dyaw -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(angular.x*(0) + angular.y*(sin(pose.rot.GetAsEuler( ).x)/cos(pose.rot.GetAsEuler( ).y)) + angular.z*(cos(pose.rot.GetAsEuler( ).x)/cos(pose.rot.GetAsEuler( ).y)));
			
			newmsg.values.push_back(juntaR->GetVelocity(0)); // daR
			newmsg.values.push_back(juntaL->GetVelocity(0)); // daL
			// publish data
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(AllData)
}
