/*
* File: pertubation.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a plugin to simulate a force pertubation.
*/

#include <pertubation.h>


namespace gazebo
{
	// constructor
	pertubation::pertubation()
	{
		
	}
	// destructor
	pertubation::~pertubation()
	{	
		
	}
	// initial setup
	void pertubation::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "Aerodinamica nao inicializado!" << std::endl;
	      		        return;
	    		}
			
			// get ROS topics name
			topicX = XMLRead::ReadXMLString("topicX",_sdf);
			topicY = XMLRead::ReadXMLString("topicY",_sdf);
			topicZ = XMLRead::ReadXMLString("topicZ",_sdf);
			// get link name
			NameOfLink = XMLRead::ReadXMLString("Link",_sdf);

			// get simulation link
			link = _model->GetLink(NameOfLink);	

			// update timer
	  		Reset();

			// subscribers			
			pertubation_subscriberX = node_handle_.subscribe(topicX, 1, &gazebo::pertubation::CallbackX, this);
			pertubation_subscriberY = node_handle_.subscribe(topicY, 1, &gazebo::pertubation::CallbackY, this);
			pertubation_subscriberZ = node_handle_.subscribe(topicZ, 1, &gazebo::pertubation::CallbackZ, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// reset
	void pertubation::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// Callback to provide a pertubation in x direction
	void pertubation::CallbackX(std_msgs::Float64 msg)
	{
		try
		{
			Fx = msg.data;
			ignition::math::Vector3d force(Fx,0,0);
			// applying
			link->AddRelativeForce(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// Callback to provide a pertubation in y direction
	void pertubation::CallbackY(std_msgs::Float64 msg)
	{
		try
		{
			Fy = msg.data;
			ignition::math::Vector3d force(0,Fy,0);
			// applying
			link->AddRelativeForce(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// Callback to provide a pertubation in z direction
	void pertubation::CallbackZ(std_msgs::Float64 msg)
	{
		try
		{
			Fz = msg.data;
			ignition::math::Vector3d force(0,0,Fz);
			// applying
			link->AddRelativeForce(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(pertubation)
}
