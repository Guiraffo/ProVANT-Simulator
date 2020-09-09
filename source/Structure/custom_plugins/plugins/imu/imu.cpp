/*
* File: gps.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement IMU. It gets information from Gazebo and quantizes the data.
*/

#include <imu.h>


namespace gazebo
{
	// constructor
	imu::imu()
	{ 
				
	}

	// destructor
	imu::~imu()
	{	
		
	}
	
	// initial setup
	void imu::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		// GET INFO XML
	  	Topic_ = XMLRead::ReadXMLString("Topic",_sdf);
		/*rpyOffset = XMLRead::ReadXMLDouble("rpyOffset",_sdf);
		rpyStandardDeviation = XMLRead::ReadXMLDouble("rpySD",_sdf);
		accelOffset = XMLRead::ReadXMLDouble("accelOffset",_sdf);
		accelStandardDeviation = XMLRead::ReadXMLDouble("accelSD",_sdf);
		angvelOffset = XMLRead::ReadXMLDouble("angvelOffset",_sdf);
		angvelStandardDeviation = XMLRead::ReadXMLDouble("angvelSD",_sdf);
		maxrpy = XMLRead::ReadXMLDouble("maxrpy",_sdf);
		minrpy = XMLRead::ReadXMLDouble("minrpy",_sdf);
		maxaccel = XMLRead::ReadXMLDouble("maxaccel",_sdf);
		minaccel = XMLRead::ReadXMLDouble("minaccel",_sdf);
		maxangvel = XMLRead::ReadXMLDouble("maxangvel",_sdf);
		minangvel = XMLRead::ReadXMLDouble("minangvel",_sdf);
		Nbits = XMLRead::ReadXMLDouble("Nbits",_sdf);*/
	  	link_name_ = XMLRead::ReadXMLString("bodyName",_sdf);
	
	  	// CREATE NOISE
	    	/*unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            	std::default_random_engine customgenerator (seed);
	    	generator = customgenerator;
            	std::normal_distribution<double> customdistributionRPY (rpyOffset,rpyStandardDeviation);
	    	std::normal_distribution<double> customdistributionACCEL (accelOffset,accelStandardDeviation);
            	std::normal_distribution<double> customdistributionANGVEL (angvelOffset,angvelStandardDeviation);
	    	distributionRPY = customdistributionRPY;
            	distributionACCEL = customdistributionACCEL;
            	distributionANGVEL = customdistributionANGVEL;*/

	 	// OTHERS TOOLS (ROS, GAZEBO)
	    	imu_pub = n.advertise<simulator_msgs::Sensor>(Topic_, 1);
		link = _model->GetLink(link_name_);
		world = _model->GetWorld();
		updateTimer.Load(world, _sdf);
	  	updateConnection = updateTimer.Connect(boost::bind(&imu::Update, this));

	}

	// reset
	void imu::Reset()
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

	// callback to publish IMU data
	void imu::Update()
	{
		try
		{
			// GET DATA
			common::Time sim_time = world->SimTime(); // simulation time
			ignition::math::Pose3d pose = link->WorldPose(); // world pose
			ignition::math::Vector3d angular = link->WorldAngularVel( ); // angular velocity
			ignition::math::Vector3d AccelLinear = link->RelativeLinearAccel(); // linear acceleration						

			// FILL MSG
			simulator_msgs::Sensor newmsg;
			newmsg.name = Topic_;
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(fmod(pose.Rot().Euler().X(),360)); // data range -pi to pi
			newmsg.values.push_back(fmod(pose.Rot().Euler().Y(),360)); // data range -pi to pi
			newmsg.values.push_back(fmod(pose.Rot().Euler().Z(),360)); // data range -pi to pi
			newmsg.values.push_back(angular.X());
			newmsg.values.push_back(angular.Y());
			newmsg.values.push_back(angular.Z());
			newmsg.values.push_back(AccelLinear.X());
			newmsg.values.push_back(AccelLinear.Y());
			newmsg.values.push_back(AccelLinear.Z());
	
			// SEND MSG
			imu_pub.publish(newmsg);	

		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	GZ_REGISTER_MODEL_PLUGIN(imu)
}
