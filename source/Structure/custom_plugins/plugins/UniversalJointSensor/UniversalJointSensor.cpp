/*
* File: UniversalJointSensor.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor the returns all kind of data enabled in the simulation of a specific joint
*/

#include <UniversalJointSensor.h>


namespace gazebo
{
	// constructor
	UniversalJointSensor::UniversalJointSensor()
	{ 
				
	}
	
	// destructor
	UniversalJointSensor::~UniversalJointSensor()
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
	void UniversalJointSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // name of topic to publish data
			NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint",_sdf); // name of joint
			axis = XMLRead::ReadXMLString("Axis",_sdf); // axis of joint
			world = _model->GetWorld(); // get pointer's world
			junta = _model->GetJoint(NameOfJoint_); // get pointer's joint
				
			// reset
	  		Reset();
			// connect with simulation time
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&UniversalJointSensor::Update, this));
			// publisher			
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// reset
	void UniversalJointSensor::Reset()
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
	void UniversalJointSensor::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock); //
			
			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now(); // time stamp
			newmsg.header.frame_id = "1";
			int index; // axis of joint
			if(axis=="axis") index = 0;
			else
			{
				if(axis=="axis2") index = 1;
				else 
				{	
					std::cout << "Erro no indice da junta, em UniversalJointSensor";
					exit(1);
				}
			}
			newmsg.values.push_back(junta->Position(index)); // get angle
			newmsg.values.push_back(junta->GetVelocity(index)); // get velocity
			newmsg.values.push_back(junta->GetForce(index)); // get force
			
			// publish data
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(UniversalJointSensor)
}
