/*
* File: QuadForces.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 10/12/19
* Description:  This library is responsable to implement aerodynamics forces in a UAV
*/

#include <QuadForces.h>

namespace gazebo
{
	// constructor
	QuadForces::QuadForces() :  F1Body(3,1), F2Body(3,1), F3Body(3,1), F4Body(3,1)
	{
		
	}
	// destructor
	QuadForces::~QuadForces()
	{	
		
	}
	// to load initial setup
	void QuadForces::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		std::cout<< "Empuxo Inicializado" <<std::endl;
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "QuadForces not initialized!" << std::endl;
	      		        return;
	    		}
			
			topic_F1 = XMLRead::ReadXMLString("topic_F1",_sdf); // name of right brushless's topic 
			topic_F2 = XMLRead::ReadXMLString("topic_F2",_sdf); // name of left brushless's topic
			topic_F3 = XMLRead::ReadXMLString("topic_F3",_sdf); // name of left brushless's topic
			topic_F4 = XMLRead::ReadXMLString("topic_F4",_sdf); // name of left brushless's topic
			NameOfLink_ = XMLRead::ReadXMLString("body",_sdf); // name of main link
			NameOfLink1_  = XMLRead::ReadXMLString("Motor1",_sdf);
			NameOfLink2_  = XMLRead::ReadXMLString("Motor2",_sdf);
			NameOfLink3_  = XMLRead::ReadXMLString("Motor3",_sdf);
			NameOfLink4_  = XMLRead::ReadXMLString("Motor4",_sdf);
			NameOfJoint1_ = XMLRead::ReadXMLString("M1",_sdf);
			NameOfJoint2_ = XMLRead::ReadXMLString("M2",_sdf);
			NameOfJoint3_ = XMLRead::ReadXMLString("M3",_sdf);
			NameOfJoint4_ = XMLRead::ReadXMLString("M4",_sdf);
			DragConst = XMLRead::ReadXMLDouble("DragCte",_sdf); //Constante de Drag
			
			
			
			// get elements of the simulation
			link = _model->GetLink(NameOfLink_);
			Motor1 = _model->GetLink(NameOfLink1_);	
			Motor2 = _model->GetLink(NameOfLink2_);
			Motor3 = _model->GetLink(NameOfLink3_);
			Motor4 = _model->GetLink(NameOfLink4_);
			
			M1 = _model->GetJoint(NameOfJoint1_);
			M2 = _model->GetJoint(NameOfJoint2_);
			M3 = _model->GetJoint(NameOfJoint3_);
			M4 = _model->GetJoint(NameOfJoint4_);
			

			// update timer
	  		Reset();

			// subscribers of data to apply in simulator			
			motor_subscriberF1_ = node_handle_.subscribe(topic_F1, 1, &gazebo::QuadForces::CallbackF1, this);
			motor_subscriberF2_ = node_handle_.subscribe(topic_F2, 1, &gazebo::QuadForces::CallbackF2, this);
			motor_subscriberF3_ = node_handle_.subscribe(topic_F3, 1, &gazebo::QuadForces::CallbackF3, this);
			motor_subscriberF4_ = node_handle_.subscribe(topic_F4, 1, &gazebo::QuadForces::CallbackF4, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// when reset simulator
	void QuadForces::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// callback to apply forces at right brushless
	void QuadForces::CallbackF1(std_msgs::Float64 msg)
	{
		try
		{

	  	           
			F1 = msg.data;
			
		//	std::cout<<"F1:"<<F1<<std::endl;
			std::cout<<"T1y:"<<-F1*0.12<< std::endl;
			
			
			ignition::math::Vector3d torque1(0,-F1*0.12*cos(0.087),DragConst*F1*cos(0.087));
			ignition::math::Vector3d Force1(-F1*sin(0.087),0,F1*cos(0.087));
         
			// Applying			
		Motor1->SetForce(Force1);
		link->SetTorque(torque1);
	//	M1->SetVelocity(0,30.0);

		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// callback to apply forces at left brushless
	void QuadForces::CallbackF2(std_msgs::Float64 msg)
	{
		try
		{	
		
			F2 = msg.data;
		//	std::cout<<"F2:"<<F2<<std::endl;
			std::cout<<"T2x:"<<F2*0.12<<std::endl;
			
			
			
			
			ignition::math::Vector3d torque2(F2*0.12*cos(0.087),0,-DragConst*F2*cos(0.087)); // drag torque				
			F2Body << 0 , 0 , F2;
      		ignition::math::Vector3d Force2(0,-F2*sin(0.087),F2*cos(0.087)); 
      
             
			// Applying			
			Motor2->SetForce(Force2);
			link->SetTorque(torque2);
	//		M2->SetVelocity(0,-30.0);
		
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void QuadForces::CallbackF3(std_msgs::Float64 msg)
	{
		try
		{
			
			F3 = msg.data;
			
		//	std::cout<<"F3:"<<F3<<std::endl;
			std::cout<<"T3y:"<<F3*0.12<< std::endl;
			
			
			F3Body << 0 , 0 , F3;
			ignition::math::Vector3d torque3(0,F3*0.12*cos(0.087),DragConst*F3*cos(0.087)); // drag torque
		
			ignition::math::Vector3d Force3(F3*sin(0.087),0,F3*cos(0.087));
			
			
			
			// Applying		
			Motor3->SetForce(Force3);
			link->SetTorque(torque3);
		//	M3->SetVelocity(0,30.0);

			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void QuadForces::CallbackF4(std_msgs::Float64 msg)
	{
		try
		{
			
			F4 = msg.data;
			
		//	std::cout<<"F4:"<<F4<<std::endl;
			std::cout<<"T4x:"<<-F4*0.12<< std::endl;
			
			
			
			ignition::math::Vector3d torque4(-F4*0.12*cos(0.087),0,-DragConst*F4*cos(0.087)); // drag torque
			F4Body << 0 , 0 , F4;		
			ignition::math::Vector3d Force4(0,F4*sin(0.087),F4*cos(0.087));
               
			// Applying			

			Motor4->SetForce(Force4);
			link->SetTorque(torque4);
	//		M4->SetVelocity(0,-30.0);
			
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	GZ_REGISTER_MODEL_PLUGIN(QuadForces)
}
