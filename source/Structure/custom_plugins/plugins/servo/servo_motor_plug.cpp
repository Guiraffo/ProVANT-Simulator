/*
* File: servo_motor_plug.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a servo motor. It works in Torque mode or Position mode and returns values of angular position and angular velocity
*/

#include <servo_motor_plug.h>


namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	// Callback for references
    void ServoMotorPlugin::CallbackReferencias(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			if(Modo_ == "Torque"){	
				if(msg.data > Force_Saturation_){
				msg.data = Force_Saturation_;
				}else if(msg.data < -Force_Saturation_){
				msg.data = -Force_Saturation_;
				}
				junta->SetForce(0,msg.data);	
			}
			if(Modo_ == "Position")	{
				if(msg.data > Angle_Saturation_){
				msg.data = Angle_Saturation_;
				}else if(msg.data < -Angle_Saturation_){
				msg.data = -Angle_Saturation_;
				}
			//std::cout << "entrei4 servo plugin!\n";
			result = junta->SetPosition(0,msg.data); // TO FIX

			}	
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// constructor
	ServoMotorPlugin::ServoMotorPlugin()
	{ 
				
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// destructor
	ServoMotorPlugin::~ServoMotorPlugin()
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// initial setup
	void ServoMotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "inicializado servo plugin!" << std::endl;
	    if (!ros::isInitialized())
	    {
			std::cout << "Nao inicializado!" << std::endl;
	    return;
	    }

			NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint",_sdf); // Get name of the joint
			TopicSubscriber_ = XMLRead::ReadXMLString("TopicSubscriber",_sdf); // Name of topic for receiving reference
			TopicPublisher_ = XMLRead::ReadXMLString("TopicPublisher",_sdf); // Name of topic for sending data
			Modo_ = XMLRead::ReadXMLString("Modo",_sdf); // mode of working defined in the model.sdf file
		//	tag_ = XMLRead::ReadXMLString("tag",_sdf);	//Tag for knowing which actuator we're saturating	
			Force_Saturation_ = XMLRead::ReadXMLDouble("Force_Saturation",_sdf);	//Value of saturation for force
			Angle_Saturation_ = XMLRead::ReadXMLDouble("Angle_Saturation",_sdf);	//Value of saturation for angle
			world = _model->GetWorld(); // get World's pointer	
			junta = _model->GetJoint(NameOfJoint_); // get joint's pointer

			// subscriber
			motor_subscriber_ = node_handle_.subscribe(TopicSubscriber_, 1, &gazebo::ServoMotorPlugin::CallbackReferencias, this);
			// publisher
			motor_publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(TopicPublisher_, 5);

	  	Reset();
			// starts connection
			updateTimer.Load(world, _sdf);
	  	updateConnection = updateTimer.Connect(boost::bind(&ServoMotorPlugin::Update, this));
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// reset
	void ServoMotorPlugin::Reset()
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************************/

	// for each ste time
	void ServoMotorPlugin::Update()
	{
		try
		{
			simulator_msgs::Sensor newmsg;
			newmsg.name = TopicPublisher_;
			newmsg.header.stamp = ros::Time::now(); // time stamp
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(junta->GetAngle(0).Radian()); // angular position
			newmsg.values.push_back(junta->GetVelocity(0)); // angular velocity
			// publishes data
			motor_publisher_.publish(newmsg);

		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	GZ_REGISTER_MODEL_PLUGIN(ServoMotorPlugin)
}
