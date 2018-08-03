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
			newmsg.values.push_back(link->GetRelativePose().pos.x);//0
                        newmsg.values.push_back(link->GetRelativePose().pos.y);//1
                        newmsg.values.push_back(link->GetRelativePose().pos.z);//2
                        newmsg.values.push_back(link->GetRelativePose().rot.GetAsEuler( ).x);//3
                        newmsg.values.push_back(link->GetRelativePose().rot.GetAsEuler( ).y);//4
                        newmsg.values.push_back(link->GetRelativePose().rot.GetAsEuler( ).z);//5
			newmsg.values.push_back(link->GetRelativeLinearVel().x);//6
                        newmsg.values.push_back(link->GetRelativeLinearVel().y);//7
                        newmsg.values.push_back(link->GetRelativeLinearVel().z);//8
			newmsg.values.push_back(link->GetRelativeLinearAccel().x);//9
                        newmsg.values.push_back(link->GetRelativeLinearAccel().y);//10
                        newmsg.values.push_back(link->GetRelativeLinearAccel().z);//11
			newmsg.values.push_back(link->GetRelativeForce().x);//12
                        newmsg.values.push_back(link->GetRelativeForce().y);//13
                        newmsg.values.push_back(link->GetRelativeForce().z);//14
			newmsg.values.push_back(link->GetRelativeAngularVel().x);//15
                        newmsg.values.push_back(link->GetRelativeAngularVel().y);//16
                        newmsg.values.push_back(link->GetRelativeAngularVel().z);//17
			newmsg.values.push_back(link->GetRelativeAngularAccel().x);//18
                        newmsg.values.push_back(link->GetRelativeAngularAccel().y);//19
                        newmsg.values.push_back(link->GetRelativeAngularAccel().z);//20
			newmsg.values.push_back(link->GetRelativeTorque().x);//21
                        newmsg.values.push_back(link->GetRelativeTorque().y);//22
                        newmsg.values.push_back(link->GetRelativeTorque().z);//23
			newmsg.values.push_back(link->GetWorldPose().pos.x);//24
                        newmsg.values.push_back(link->GetWorldPose().pos.y);//25
                        newmsg.values.push_back(link->GetWorldPose().pos.z);//26
                        newmsg.values.push_back(link->GetWorldPose().rot.GetAsEuler( ).x);//27
                        newmsg.values.push_back(link->GetWorldPose().rot.GetAsEuler( ).y);//28
                        newmsg.values.push_back(link->GetWorldPose().rot.GetAsEuler( ).z);//29
			newmsg.values.push_back(link->GetWorldLinearVel().x);//30
                        newmsg.values.push_back(link->GetWorldLinearVel().y);//31
                        newmsg.values.push_back(link->GetWorldLinearVel().z);//32
			newmsg.values.push_back(link->GetWorldLinearAccel().x);//33
                        newmsg.values.push_back(link->GetWorldLinearAccel().y);//34
                        newmsg.values.push_back(link->GetWorldLinearAccel().z);//35
			newmsg.values.push_back(link->GetWorldForce().x);//36
                        newmsg.values.push_back(link->GetWorldForce().y);//37
                        newmsg.values.push_back(link->GetWorldForce().z);//38
			newmsg.values.push_back(link->GetWorldAngularVel().x);//39
                        newmsg.values.push_back(link->GetWorldAngularVel().y);//40
                        newmsg.values.push_back(link->GetWorldAngularVel().z);//41
			newmsg.values.push_back(link->GetWorldAngularAccel().x);//42
                        newmsg.values.push_back(link->GetWorldAngularAccel().y);//43
                        newmsg.values.push_back(link->GetWorldAngularAccel().z);//44
			newmsg.values.push_back(link->GetWorldTorque().x);//45
                        newmsg.values.push_back(link->GetWorldTorque().y);//46
                        newmsg.values.push_back(link->GetWorldTorque().z);//47
			newmsg.values.push_back(link->GetWorldCoGLinearVel().x);//48
                        newmsg.values.push_back(link->GetWorldCoGLinearVel().y);//49
                        newmsg.values.push_back(link->GetWorldCoGLinearVel().z);//50
			newmsg.values.push_back(link->GetWorldCoGPose().pos.x);//51
                        newmsg.values.push_back(link->GetWorldCoGPose().pos.y);//52
                        newmsg.values.push_back(link->GetWorldCoGPose().pos.z);//53
			
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
