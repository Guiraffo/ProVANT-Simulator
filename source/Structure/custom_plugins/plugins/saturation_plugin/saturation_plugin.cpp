/*
* File: servo_motor_plug.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a servo motor. It works in Torque mode or Position mode and returns values of angular position and angular velocity
*/

#include <saturation_plugin.h>


namespace gazebo
{
	
	// Callback for references
    void SaturationPlugin::CallbackReferencias1(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			
				outsfileRotR.printFile(msg.data);
			
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}
	
	void SaturationPlugin::CallbackReferencias2(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			
				outsfileRotL.printFile(msg.data);
			
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}
	
	void SaturationPlugin::CallbackReferencias3(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			
				outsfileAilR.printFile(msg.data);
			
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}
	
	void SaturationPlugin::CallbackReferencias4(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			
				outsfileAilL.printFile(msg.data);
			
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}
	
	void SaturationPlugin::CallbackReferencias5(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			
				outsfileRudR.printFile(msg.data);
			
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}
	
	void SaturationPlugin::CallbackReferencias6(std_msgs::Float64 msg)
	{
		//std::cout << "entreiiiiii servo plugin!\n";
		try
		{	bool result = false;
			//std::cout << "entrei2 servo plugin!\n";
			
				outsfileRudL.printFile(msg.data);
			
		}
		catch(std::exception& e)
		{
			//std::cout << "entrei3 servo plugin!\n";
			std::cout << e.what() << std::endl;
		}
	}

	// constructor
	SaturationPlugin::SaturationPlugin()
	{ 
	
				teste doc(std::getenv("TILT_CONFIG"));
				
				docme=doc;
				
				std::string OutputsaturedPathRotR = docme.GetItem("OutputsaturedfileRotR");
				std::string OutputsaturedPathRotL = docme.GetItem("OutputsaturedfileRotL");
				std::string OutputsaturedPathAilR = docme.GetItem("OutputsaturedfileAilR");
				std::string OutputsaturedPathAilL = docme.GetItem("OutputsaturedfileAilL");
				std::string OutputsaturedPathRudR = docme.GetItem("OutputsaturedfileRudR");
				std::string OutputsaturedPathRudL = docme.GetItem("OutputsaturedfileRudL");
				
				OutputsaturedPathRotR = std::getenv("TILT_MATLAB") + OutputsaturedPathRotR;
				OutputsaturedPathRotL = std::getenv("TILT_MATLAB") + OutputsaturedPathRotL;
				OutputsaturedPathAilR = std::getenv("TILT_MATLAB") + OutputsaturedPathAilR;
				OutputsaturedPathAilL = std::getenv("TILT_MATLAB") + OutputsaturedPathAilL;
				OutputsaturedPathRudR = std::getenv("TILT_MATLAB") + OutputsaturedPathRudR;
				OutputsaturedPathRudL = std::getenv("TILT_MATLAB") + OutputsaturedPathRudL;
				
				
				outsfileRotR.startFile(OutputsaturedPathRotR,"OUTSROTR");
				outsfileRotL.startFile(OutputsaturedPathRotL,"OUTSROTL");
				outsfileAilR.startFile(OutputsaturedPathAilR,"OUTSAILR");
				outsfileAilL.startFile(OutputsaturedPathAilL,"OUTSAILL");
				outsfileRudR.startFile(OutputsaturedPathRudR,"OUTSRUDR");
				outsfileRudL.startFile(OutputsaturedPathRudL,"OUTSRUDL");
				
	}

	// destructor
	SaturationPlugin::~SaturationPlugin()
	{	
		outsfileRotR.endFile();
		outsfileRotL.endFile();
		outsfileAilR.endFile();
		outsfileAilL.endFile();
		outsfileRudR.endFile();
		outsfileRudL.endFile();
		
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
	void SaturationPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "inicializado saturacao plugin!" << std::endl;
	    		if (!ros::isInitialized())
	    		{
				std::cout << "Nao inicializado!" << std::endl;
	      		        return;
	    		}


			TopicSubscriber1_ = XMLRead::ReadXMLString("TopicSubscriber1",_sdf); // Name of topic for receiving reference
			TopicSubscriber2_ = XMLRead::ReadXMLString("TopicSubscriber2",_sdf); // Name of topic for receiving reference
			TopicSubscriber3_ = XMLRead::ReadXMLString("TopicSubscriber3",_sdf); // Name of topic for receiving reference
			TopicSubscriber4_ = XMLRead::ReadXMLString("TopicSubscriber4",_sdf); // Name of topic for receiving reference
			TopicSubscriber5_ = XMLRead::ReadXMLString("TopicSubscriber5",_sdf); // Name of topic for receiving reference
			TopicSubscriber6_ = XMLRead::ReadXMLString("TopicSubscriber6",_sdf); // Name of topic for receiving reference
			
			
			subscriber1_ = node_handle_.subscribe(TopicSubscriber1_, 1, &gazebo::SaturationPlugin::CallbackReferencias1, this);
			subscriber2_ = node_handle_.subscribe(TopicSubscriber2_, 1, &gazebo::SaturationPlugin::CallbackReferencias2, this);
			subscriber3_ = node_handle_.subscribe(TopicSubscriber3_, 1, &gazebo::SaturationPlugin::CallbackReferencias3, this);
			subscriber4_ = node_handle_.subscribe(TopicSubscriber4_, 1, &gazebo::SaturationPlugin::CallbackReferencias4, this);
			subscriber5_ = node_handle_.subscribe(TopicSubscriber5_, 1, &gazebo::SaturationPlugin::CallbackReferencias5, this);
			subscriber6_ = node_handle_.subscribe(TopicSubscriber6_, 1, &gazebo::SaturationPlugin::CallbackReferencias6, this);
			
	  		Reset();
			// starts connection
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&SaturationPlugin::Update, this));
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	// reset
	void SaturationPlugin::Reset()
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
	
	// for each ste time
	void SaturationPlugin::Update()
	{
		try
		{
		common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);
		std::cout << "ESTOU NO UPDATE" << std::endl;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	
	
	
	GZ_REGISTER_MODEL_PLUGIN(SaturationPlugin)
}
