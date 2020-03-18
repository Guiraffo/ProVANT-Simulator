/*
* File: QuadForces.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 10/12/19
* Description:  This library is responsable to implement aerodynamics forces in a UAV of type quadrotor
*/

#include <QuadForces.h>

namespace gazebo
{
/*****************************************************************************************************************************/
	// constructor
	QuadForces::QuadForces() :  ForceBody(3),TorqueBody(3) 
	
	{
	  //till angle of the motors
		alpha = 0.087;
		//length of the quadcopter's arms
		length = 0.332; 
		
	}


/*****************************************************************************************************************************/

	//Destructor
	QuadForces::~QuadForces()
	{	//Wind.endFile();
		try
		{
			updateTimer.Disconnect(updateConnection);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}
	
	
/*****************************************************************************************************************************/

	
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
			
			topic_F1 = XMLRead::ReadXMLString("topic_F1",_sdf); // name of brushless number 1 
			topic_F2 = XMLRead::ReadXMLString("topic_F2",_sdf); // name of brushless number 2 
			topic_F3 = XMLRead::ReadXMLString("topic_F3",_sdf); // name of brushless number 3 
			topic_F4 = XMLRead::ReadXMLString("topic_F4",_sdf); // name of brushless number 4 
			NameOfLink_ = XMLRead::ReadXMLString("body",_sdf); // name of main link
			DragConst = XMLRead::ReadXMLDouble("DragCte",_sdf); //Drag constant
			
			
			
			
			// get elements of the simulation
			link = _model->GetLink(NameOfLink_);
			// get world pointer
 			world = _model->GetWorld();
		
			// update timer
	  		Reset();	  		
			updateTimer.Load(world, _sdf);
			updateConnection = updateTimer.Connect(boost::bind(&QuadForces::Update, this));

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


/*****************************************************************************************************************************/


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


/*****************************************************************************************************************************/


	// callback to get the force at brushless 1
	void QuadForces::CallbackF1(std_msgs::Float64 msg)
	{
		try
		{  	           
			F1 = msg.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


/*****************************************************************************************************************************/
	
	
	// callback to get the force at brushless 2
	void QuadForces::CallbackF2(std_msgs::Float64 msg)
	{
		try
		{	
			F2 = msg.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	


/*****************************************************************************************************************************/

	// callback to get the force at brushless 3
	void QuadForces::CallbackF3(std_msgs::Float64 msg)
	{
		try
		{
			F3 = msg.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


/*****************************************************************************************************************************/

	// callback to get the force at brushless 4
	void QuadForces::CallbackF4(std_msgs::Float64 msg)
	{
		try
		{
			F4 = msg.data;	
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


/*****************************************************************************************************************************/



void QuadForces::Update(){
			

			TorqueBody << (F2-F4)*length*cos(alpha),(F3-F1)*length*cos(alpha),DragConst*(F1 + F3 - F2 - F4)*cos(alpha);
			ForceBody << (F3-F1)*sin(alpha) , (F4-F2)*sin(alpha) , (F1+F2+F3+F4)*cos(alpha);
		
			math::Vector3 Torque(TorqueBody(0),TorqueBody(1),TorqueBody(2));
			math::Vector3 Force(ForceBody(0),ForceBody(1),ForceBody(2));

			//Applies force and torque w.r.t the body frame
			link->AddRelativeForce(Force);
			link->AddRelativeTorque(Torque);
			

}

	GZ_REGISTER_MODEL_PLUGIN(QuadForces)
}
