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
#include <eigen3/Eigen/Eigen>

namespace gazebo
{
	// constructor
	AllData::AllData(): PhipThetapPsip(3,1), RIB(3,3), W_n(3,3), WIIB(3,1)
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
			ignition::math::Pose3d pose = link->WorldPose();
			newmsg.values.push_back(pose.Pos().X()); // x
			newmsg.values.push_back(pose.Pos().Y()); // y
			newmsg.values.push_back(pose.Pos().Z()); // z
			newmsg.values.push_back(pose.Rot().Euler().X()); //roll
			newmsg.values.push_back(pose.Rot().Euler().Y()); // pitch
			newmsg.values.push_back(pose.Rot().Euler().Z()); // yaw
			newmsg.values.push_back(juntaR->Position(0)); //aR
			newmsg.values.push_back(juntaL->Position(0)); //aL
			ignition::math::Vector3d linear = link->WorldLinearVel();
			newmsg.values.push_back(linear.X()); // vx
			newmsg.values.push_back(linear.Y()); // vy
			newmsg.values.push_back(linear.Z()); //vz
			ignition::math::Vector3d angular = link->WorldAngularVel( );
			
			
			//antigo
			/*
			newmsg.values.push_back(angular.X()); //11
			newmsg.values.push_back(angular.Y()); //12
			newmsg.values.push_back(angular.Z()); //13
			
						// droll -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(angular.X()*(1) + angular.Y()*((sin(pose.Rot().Euler().X())*sin(pose.Rot().Euler().Y()))/cos(pose.Rot().Euler().Y())) + angular.Z()*((cos(pose.Rot().Euler().X())*sin(pose.Rot().Euler().Y()))/cos(pose.Rot().Euler().Y())));
			// dpitch  -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(angular.X()*(0) + angular.Y()*(cos(pose.Rot().Euler().X())) + angular.Z()*(-sin(pose.Rot().Euler().X())));
			// dyaw -> attention! we receive angular velocity, but we want to publish the derivative of euler angle
			newmsg.values.push_back(angular.X()*(0) + angular.Y()*(sin(pose.Rot().Euler().X())/cos(pose.Rot().Euler().Y())) + angular.Z()*(cos(pose.Rot().Euler().X())/cos(pose.Rot().Euler().Y())));
			*/
			
			//novo			
			//Maps to the body
			Phi = pose.Rot().Euler().X();
			Theta = pose.Rot().Euler().Y();
			Psi = pose.Rot().Euler().Z();
			
			RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));
	
			W_n << 1.0,         0.0,          -sin(Theta), 
			       0.0,  cos(Phi),  cos(Theta)*sin(Phi),
	  	               0.0, -sin(Phi),  cos(Phi)*cos(Theta);
	
			WIIB << angular.X(), angular.Y(), angular.Z();
			PhipThetapPsip = W_n.inverse() * RIB.transpose() * WIIB;
			
			newmsg.values.push_back(PhipThetapPsip(0));
			newmsg.values.push_back(PhipThetapPsip(1));
			newmsg.values.push_back(PhipThetapPsip(2));
			
			newmsg.values.push_back(juntaR->GetVelocity(0)); // daR
			newmsg.values.push_back(juntaL->GetVelocity(0)); // daL
			//std::cout << "alldata" << std::endl;
			
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
