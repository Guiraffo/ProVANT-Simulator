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
	QuadForces::QuadForces() : RIB(3,3) , F1Body(3,1), F2Body(3,1), F3Body(3,1), F4Body(3,1), F1Inertial(3,1), F2Inertial(3,1), F3Inertial(3,1), F4Inertial(3,1)
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
			NameOfJoint1_  = XMLRead::ReadXMLString("Motor1",_sdf);
			NameOfJoint2_  = XMLRead::ReadXMLString("Motor2",_sdf);
			NameOfJoint3_  = XMLRead::ReadXMLString("Motor3",_sdf);
			NameOfJoint4_  = XMLRead::ReadXMLString("Motor4",_sdf);
			DragConst = XMLRead::ReadXMLDouble("DragCte",_sdf); //Constante de Drag
			
			
			
			// get elements of the simulation
			link = _model->GetLink(NameOfLink_);
			Motor1 = _model->GetJoint(NameOfJoint1_);	
			Motor2 = _model->GetJoint(NameOfJoint2_);
			Motor3 = _model->GetJoint(NameOfJoint3_);
			Motor4 = _model->GetJoint(NameOfJoint4_);

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
			std::cout<<"F1:"<<F1<<std::endl;
			RotationMatrixInertialBody();
			ignition::math::Vector3d torque1(0,0,-DragConst*F1);
			ignition::math::Vector3d RelativePose1(0.12,0,0);  //0.12,0,0
			F1Body << 0 , 0 , F1;
			F1Inertial = RIB*F1Body;		
             
			// Applying			
			link->AddForceAtRelativePosition(ignition::math::Vector3d(F1Body(0),F1Body(1),F1Body(2)),RelativePose1);
			link->SetTorque(torque1);
		//	Motor1->SetVelocity(0,10.0);

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
			RotationMatrixInertialBody();
			std::cout<<"F2:"<<F2<<std::endl;
			ignition::math::Vector3d torque2(0,0,DragConst*F2); // drag torque				
			ignition::math::Vector3d RelativePose2(0,0.12,0);	// Applying	 //0,0.12,0
			
			F2Body << 0 , 0 , F2;
			F2Inertial = RIB*F2Body;	 	    
               
			// Applying			
			link->AddForceAtRelativePosition(ignition::math::Vector3d(F2Body(0),F2Body(1),F2Body(2)),RelativePose2);
			link->SetTorque(torque2);
		//	Motor2->SetVelocity(0,-10.0);
		
			
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
			std::cout<<"F3:"<<F3<<std::endl;
			RotationMatrixInertialBody();
			F3Body << 0 , 0 , F3;
			ignition::math::Vector3d torque3(0,0,-DragConst*F3); // drag torque
			ignition::math::Vector3d RelativePose3(-0.12,0,0);		//-0.12,0,0
			// Applying		
			F3Inertial = RIB*F3Body;
			link->AddForceAtRelativePosition(ignition::math::Vector3d(F3Body(0),F3Body(1),F3Body(2)),RelativePose3);
			link->SetTorque(torque3);
		//	Motor3->SetVelocity(0,10.0);

			
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
			std::cout<<"F4:"<<F4<<std::endl;
			RotationMatrixInertialBody();
			ignition::math::Vector3d torque4(0,0,DragConst*F4); // drag torque
			ignition::math::Vector3d RelativePose4(0,-0.12,0);	//
			
			F4Body << 0 , 0 , F4;
			F4Inertial = RIB*F4Body;		
			
               
			// Applying			
			link->AddForceAtRelativePosition(ignition::math::Vector3d(F4Body(0),F4Body(1),F4Body(2)),RelativePose4);
			link->SetTorque(torque4);
		//	Motor4->SetVelocity(0,-10.0);
			
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void QuadForces::RotationMatrixInertialBody(){
	
	double Phi, Theta, Psi;
	math::Pose pose = link->WorldPose();
	Phi = pose.Rot().Euler( ).X();
	Theta = pose.Rot().Euler( ).Y();
	Psi = pose.Rot().Euler( ).Z();
	RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));
      
	}

	GZ_REGISTER_MODEL_PLUGIN(QuadForces)
}
