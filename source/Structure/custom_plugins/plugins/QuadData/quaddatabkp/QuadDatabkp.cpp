/*
* File: QuadData
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 13/02/19
* Description:  This library is responsable to implement a sensor that return for a Quadrotor all data of the folowing state space:

- x,y,z,roll,pitch,yaw,dx,dy,dz,droll,dpitch,dyaw

*/


#include <QuadData.h>


namespace gazebo
{
	// constructor
	QuadData::QuadData()
	{ 
				
	}

	// destructor
	QuadData::~QuadData()
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
	void QuadData::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "entrei Quad Estados plugin"<<std::endl;
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
	  		updateConnection = updateTimer.Connect(boost::bind(&QuadData::Update, this));
			// ROS publisher
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);  //NameofNode é o nome do Topico na verdade
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// reset
	void QuadData::Reset()
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
	void QuadData::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock); // mutex
			
			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now(); // time stamp
			newmsg.header.frame_id = "1";
			ignition::math::Pose3d pose = link->WorldPose();
			newmsg.values.push_back(pose.Pos().X()); // x
			newmsg.values.push_back(pose.Pos().Y()); // y
			newmsg.values.push_back(pose.Pos().Z()); // z
			newmsg.values.push_back(pose.Rot().Euler().X()); //roll
			newmsg.values.push_back(pose.Rot().Euler().Y()); // pitch
			newmsg.values.push_back(pose.Rot().Euler().Z()); // yaw
			ignition::math::Vector3d linear = link->WorldLinearVel();
			newmsg.values.push_back(linear.X()); // dx
			newmsg.values.push_back(linear.Y()); // dy
			newmsg.values.push_back(linear.Z()); //dz
			gnition::math::Vector3d angular = link->WorldAngularVel( );
			// droll -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			double phi = pose.Rot().Euler().X();
			double theta = pose.Rot().Euler().Y();
			double psi = pose.Rot().Euler().Z();
			double p = angular.X();
			double q = angular.Y();
			double r = angular.Z();
			newmsg.values.push_back(p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)); 
			 // dpitch  -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(0 + q*cos(phi) - r*sin(phi));    
			// dyaw -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(0 + q*sin(phi)*(1/cos(theta)) + r*cos(phi)*(1/cos(theta)));
			
			
			// publish data
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
			
			

			

	GZ_REGISTER_MODEL_PLUGIN(QuadData)
}
