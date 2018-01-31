/*
* File: force.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a force in a speficific link of UAV
*/

#include <force.h>

//using namespace gazebo::math;

namespace gazebo
{
	// constructor
	force::force()
	{
		
	}

	// destructor
	force::~force()
	{	
		
	}
	// initial setup
	void force::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "Aerodinamica nao inicializado!" << std::endl;
	      		        return;
	    		}
			
			topicX = XMLRead::ReadXMLString("topicX",_sdf); // get force in x
			topicY = XMLRead::ReadXMLString("topicY",_sdf); // get force in y
			topicZ = XMLRead::ReadXMLString("topicZ",_sdf); // get force in z
			NameOfLink = XMLRead::ReadXMLString("Link",_sdf); // get name of link

			link = _model->GetLink(NameOfLink);// get name if link

			// update timer
	  		Reset();

			// subscribers			
			pertubation_subscriberX = node_handle_.subscribe(topicX, 1, &gazebo::force::CallbackX, this);
			pertubation_subscriberY = node_handle_.subscribe(topicY, 1, &gazebo::force::CallbackY, this);
			pertubation_subscriberZ = node_handle_.subscribe(topicZ, 1, &gazebo::force::CallbackZ, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	//reset
	void force::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// callback of force in x
	void force::CallbackX(std_msgs::Float64 msg)
	{
		try
		{
			//Applying force
			Fx = msg.data;
			math::Vector3 force(Fx,0,0);
			link->AddRelativeForce(force); // relative force
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// callback of force in Y
	void force::CallbackY(std_msgs::Float64 msg)
	{
		try
		{
			//Applying force
			Fy = msg.data;
			math::Vector3 force(0,Fy,0);
			link->AddRelativeForce(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// callback of force in Z
	void force::CallbackZ(std_msgs::Float64 msg)
	{
		try
		{
			//Applying force
			Fz = msg.data;
			math::Vector3 force(0,0,Fz);
			link->AddRelativeForce(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(force)
}
