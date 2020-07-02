/*
* File: UniversalLinkSensor.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor the returns all kind of data enabled in the simulation of a specific link
*/

#include <UniversalLinkSensor.h>


namespace gazebo
{
	// constructor
	UniversalLinkSensor::UniversalLinkSensor()
	{ 
				
	}
	// destructor
	UniversalLinkSensor::~UniversalLinkSensor()
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
	void UniversalLinkSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // name of topic to publish data
			world = _model->GetWorld();	 // get world
			link_name_ = XMLRead::ReadXMLString("NameOfLink",_sdf); // get name oh link
			link = _model->GetLink(link_name_); // get link			

			// reset 
	  		Reset();
			// connect with simulation time
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&UniversalLinkSensor::Update, this));
			// publisher			
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	// reset
	void UniversalLinkSensor::Reset()
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
	// for each step time
	void UniversalLinkSensor::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock);
			
			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now(); // time stamp
			newmsg.header.frame_id = "1";
			// more information http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Link.html
			newmsg.values.push_back(link->RelativePose().Pos().X());//0
                        newmsg.values.push_back(link->RelativePose().Pos().Y());//1
                        newmsg.values.push_back(link->RelativePose().Pos().Z());//2
                        newmsg.values.push_back(link->RelativePose().Rot().Euler().X());//3
                        newmsg.values.push_back(link->RelativePose().Rot().Euler().Y());//4
                        newmsg.values.push_back(link->RelativePose().Rot().Euler().Z());//5
			newmsg.values.push_back(link->RelativeLinearVel().X());//6
                        newmsg.values.push_back(link->RelativeLinearVel().Y());//7
                        newmsg.values.push_back(link->RelativeLinearVel().Z());//8
			newmsg.values.push_back(link->RelativeLinearAccel().X());//9
                        newmsg.values.push_back(link->RelativeLinearAccel().Y());//10
                        newmsg.values.push_back(link->RelativeLinearAccel().Z());//11
			newmsg.values.push_back(link->RelativeForce().X());//12
                        newmsg.values.push_back(link->RelativeForce().Y());//13
                        newmsg.values.push_back(link->RelativeForce().Z());//14
			newmsg.values.push_back(link->RelativeAngularVel().X());//15
                        newmsg.values.push_back(link->RelativeAngularVel().Y());//16
                        newmsg.values.push_back(link->RelativeAngularVel().Z());//17
			newmsg.values.push_back(link->RelativeAngularAccel().X());//18
                        newmsg.values.push_back(link->RelativeAngularAccel().Y());//19
                        newmsg.values.push_back(link->RelativeAngularAccel().Z());//20
			newmsg.values.push_back(link->RelativeTorque().X());//21
                        newmsg.values.push_back(link->RelativeTorque().Y());//22
                        newmsg.values.push_back(link->RelativeTorque().Z());//23
			newmsg.values.push_back(link->WorldPose().Pos().X());//25
                        newmsg.values.push_back(link->WorldPose().Pos().Y());//25
                        newmsg.values.push_back(link->WorldPose().Pos().Z());//26
                        newmsg.values.push_back(link->WorldPose().Rot().Euler().X());//27
                        newmsg.values.push_back(link->WorldPose().Rot().Euler().Y());//28
                        newmsg.values.push_back(link->WorldPose().Rot().Euler().Z());//29
			newmsg.values.push_back(link->WorldLinearVel().X());//30
                        newmsg.values.push_back(link->WorldLinearVel().Y());//31
                        newmsg.values.push_back(link->WorldLinearVel().Z());//32
			newmsg.values.push_back(link->WorldLinearAccel().X());//33
                        newmsg.values.push_back(link->WorldLinearAccel().Y());//34
                        newmsg.values.push_back(link->WorldLinearAccel().Z());//35
			newmsg.values.push_back(link->WorldForce().X());//36
                        newmsg.values.push_back(link->WorldForce().Y());//37
                        newmsg.values.push_back(link->WorldForce().Z());//38
			newmsg.values.push_back(link->WorldAngularVel().X());//39
                        newmsg.values.push_back(link->WorldAngularVel().Y());//40
                        newmsg.values.push_back(link->WorldAngularVel().Z());//41
			newmsg.values.push_back(link->WorldAngularAccel().X());//42
                        newmsg.values.push_back(link->WorldAngularAccel().Y());//43
                        newmsg.values.push_back(link->WorldAngularAccel().Z());//44
			newmsg.values.push_back(link->WorldTorque().X());//45
                        newmsg.values.push_back(link->WorldTorque().Y());//46
                        newmsg.values.push_back(link->WorldTorque().Z());//47
			newmsg.values.push_back(link->WorldCoGLinearVel().X());//48
                        newmsg.values.push_back(link->WorldCoGLinearVel().Y());//49
                        newmsg.values.push_back(link->WorldCoGLinearVel().Z());//50
			newmsg.values.push_back(link->WorldCoGPose().Pos().X());//51
                        newmsg.values.push_back(link->WorldCoGPose().Pos().Y());//52
                        newmsg.values.push_back(link->WorldCoGPose().Pos().Z());//53
			
			// publish data
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(UniversalLinkSensor)
}
