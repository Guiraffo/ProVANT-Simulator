/*
* File: aerodinamica.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement aerodynamics forces in a UAV
*/

#include <aerodinamica.h>

namespace gazebo
{
	// constructor
	Aerodinamica::Aerodinamica()
	{
		
	}
	// destructor
	Aerodinamica::~Aerodinamica()
	{	
		
	}
	// to load initial setup
	void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		std::cout<< "Empuxo Inicializado" <<std::endl;
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "Aerodynamics not initialized!" << std::endl;
	      		        return;
	    		}
			
			topic_FR = XMLRead::ReadXMLString("topic_FR",_sdf); // name of right brushless's topic 
			topic_FL = XMLRead::ReadXMLString("topic_FL",_sdf); // name of left brushless's topic
			NameOfLinkDir_ = XMLRead::ReadXMLString("LinkDir",_sdf); // name of right brushless's link
			NameOfLinkEsq_ = XMLRead::ReadXMLString("LinkEsq",_sdf); // name of left brushless's link
			DragConst = XMLRead::ReadXMLDouble("DragCte",_sdf); //Constante de Drag
			
			
			
			// get elements of the simulation
			linkR = _model->GetLink(NameOfLinkDir_);
			linkL = _model->GetLink(NameOfLinkEsq_);	

			// update timer
	  		Reset();

			// subscribers of data to apply in simulator			
			motor_subscriberFR_ = node_handle_.subscribe(topic_FR, 1, &gazebo::Aerodinamica::CallbackFR, this);
			motor_subscriberFL_ = node_handle_.subscribe(topic_FL, 1, &gazebo::Aerodinamica::CallbackFL, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// when reset simulator
	void Aerodinamica::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	// callback to apply forces at right brushless
	void Aerodinamica::CallbackFR(std_msgs::Float64 msg)
	{
		try
		{
			
			Fr = msg.data;
			std::cout<<"Fr:"<<Fr<<std::endl;
			math::Vector3 forceR(0,0,Fr); // Right force in the left brushless
			math::Vector3 torqueR(0,0,DragConst*Fr); // drag torque
			// Applying			
			linkR->AddRelativeForce(forceR);
			linkR->AddRelativeTorque(torqueR);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	// callback to apply forces at left brushless
	void Aerodinamica::CallbackFL(std_msgs::Float64 msg)
	{
		try
		{	
		
			Fl = msg.data;
			std::cout<<"Fl:"<<Fl<<std::endl;
			math::Vector3 forceL(0,0,Fl); // Lift force in the left brushless
			math::Vector3 torqueL(0,0,-DragConst*Fl); // drag torque				
			// Applying			
			linkL->AddRelativeForce(forceL);
			linkL->AddRelativeTorque(torqueL);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	

	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}
