/*
* File: QuadForces.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 10/12/19
* Description:  This library is responsable to implement aerodynamics forces in a UAV
*/





/****************************************************THIS VERSION WORKS BUT IS NOT THE BEST************************************************/




#include <QuadForces.h>

namespace gazebo
{
	// constructor
	QuadForces::QuadForces() :  F1Body(3,1), F2Body(3,1), F3Body(3,1), F4Body(3,1) , ForceInertial(3),  ForceBody(3), RIB(3,3)
	{
		alpha = 0.087;
		length = 0.12; 
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
			DragConst = XMLRead::ReadXMLDouble("DragCte",_sdf); //Constante de Drag
			
			
			
			// get elements of the simulation
			link = _model->GetLink(NameOfLink_);
			Motor1 = _model->GetLink(NameOfLink1_);	
			Motor2 = _model->GetLink(NameOfLink2_);
			Motor3 = _model->GetLink(NameOfLink3_);
			Motor4 = _model->GetLink(NameOfLink4_);
			

			

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
		  ApplyForces("F1",F1);
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
			ApplyForces("F2",F2);	
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
		  ApplyForces("F3",F3);
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
			ApplyForces("F4",F4);		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void QuadForces::ApplyForces(std::string tag, double forca){



			if(tag == "F1"){
					F1 = forca;
			}	else if(tag == "F2"){
				 F2 = forca;
			}else if(tag=="F3"){
				F3 = forca;
			}else if(tag == "F4"){
				F4 = forca;
			}
			//std::cout << "F1: " << F1 << std::endl;
			//std::cout << "F2: " << F2 << std::endl; 
			//std::cout << "F3: " << F3 << std::endl;
			//std::cout << "F4: " << F4 << std::endl;


			math::Vector3 Torque((F2-F4)*length*cos(alpha),(F3-F1)*length*cos(alpha),(DragConst*F1 + DragConst*F3 - DragConst*F2 - DragConst*F4)*cos(alpha));

			math::Pose pose = link->GetWorldPose();
			Phi = pose.rot.GetAsEuler( ).x;
			Theta = pose.rot.GetAsEuler( ).y;
			Psi = pose.rot.GetAsEuler( ).z;
			
			RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));


			ForceBody << (F3-F1)*sin(alpha) , (F4-F2)*sin(alpha) , (F1+F2+F3+F4)*cos(alpha);
			ForceInertial = RIB*ForceBody;			// Force must be expressed in Inertial frame in order to use SetForce(math::Vector3 Force))
			math::Vector3 Force(ForceInertial(0),ForceInertial(1),ForceInertial(2)); 

			link->SetForce(Force);		
			link->SetTorque(Torque);

}

	GZ_REGISTER_MODEL_PLUGIN(QuadForces)
}
