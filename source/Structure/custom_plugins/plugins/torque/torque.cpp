/*
* File: torque.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement torque actuator with one step time duration
*/

#include <torque.h>

namespace gazebo
{

	// constructor
	torque::torque()
	{
		
	}

	// destructor
	torque::~torque()
	{	
		
	}

	// initial setup
	void torque::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "Aerodinamica nao inicializado!" << std::endl;
	      		        return;
	    		}
			
			topicX = XMLRead::ReadXMLString("topicX",_sdf); // get the name of topic of torque's value to be applied in x
			topicY = XMLRead::ReadXMLString("topicY",_sdf); // get the name of topic of torque's value to be applied in y
			topicZ = XMLRead::ReadXMLString("topicZ",_sdf); // get the name of topic of torque's value to be applied in z
			NameOfLink = XMLRead::ReadXMLString("Link",_sdf); // get the name of link to be applied the torque

			// get link
			link = _model->GetLink(NameOfLink);	

			// reset
	  		Reset();

			// subscribers			
			pertubation_subscriberX = node_handle_.subscribe(topicX, 1, &gazebo::torque::CallbackX, this);
			pertubation_subscriberY = node_handle_.subscribe(topicY, 1, &gazebo::torque::CallbackY, this);
			pertubation_subscriberZ = node_handle_.subscribe(topicZ, 1, &gazebo::torque::CallbackZ, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// reset
	void torque::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// receive data of torque in x
	void torque::CallbackX(std_msgs::Float64 msg)
	{
		try
		{
			Fx = msg.data;
			math::Vector3 force(Fx,0,0);
			link->AddRelativeTorque(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// receive data of torque in y
	void torque::CallbackY(std_msgs::Float64 msg)
	{
		try
		{
			Fy = msg.data;
			math::Vector3 force(0,Fy,0);
			link->AddRelativeTorque(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// receive data of torque in z
	void torque::CallbackZ(std_msgs::Float64 msg)
	{
		try
		{
			Fz = msg.data;
			math::Vector3 force(0,0,Fz);
			link->AddRelativeTorque(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(torque)
}
