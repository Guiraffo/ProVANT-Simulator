#include <imu.h>


namespace gazebo
{
	imu::imu()
	{ 
				
	}

	imu::~imu()
	{	
		
	}

	void imu::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		// GET INFO XML
	  	Topic_ = XMLRead::ReadXMLString("Topic",_sdf);
		rpyOffset = XMLRead::ReadXMLDouble("rpyOffset",_sdf);
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
		Nbits = XMLRead::ReadXMLDouble("Nbits",_sdf);
	  	link_name_ = XMLRead::ReadXMLString("bodyName",_sdf);
	
	  	// CREATE NOISE
	    	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            	std::default_random_engine customgenerator (seed);
	    	generator = customgenerator;
            	std::normal_distribution<double> customdistributionRPY (rpyOffset,rpyStandardDeviation);
	    	std::normal_distribution<double> customdistributionACCEL (accelOffset,accelStandardDeviation);
            	std::normal_distribution<double> customdistributionANGVEL (angvelOffset,angvelStandardDeviation);
	    	distributionRPY = customdistributionRPY;
            	distributionACCEL = customdistributionACCEL;
            	distributionANGVEL = customdistributionANGVEL;

	 	// OTHERS TOOLS (ROS, GAZEBO)
	    	imu_pub = n.advertise<simulator_msgs::Sensor>(Topic_, 1);
		link = _model->GetLink(link_name_);
		world = _model->GetWorld();
		updateTimer.Load(world, _sdf);
	  	updateConnection = updateTimer.Connect(boost::bind(&imu::Update, this));

	}

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

	void imu::Update()
	{
		try
		{
			// GET DATA
			common::Time sim_time = world->GetSimTime();
			math::Pose pose = link->GetWorldPose();
			math::Vector3 angular = link->GetWorldAngularVel( );
			math::Vector3 AccelLinear = link->GetRelativeLinearAccel();						

			// FILL MSG
			simulator_msgs::Sensor newmsg;
			newmsg.name = Topic_;
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(fmod(pose.rot.GetAsEuler( ).x,360));
			newmsg.values.push_back(fmod(pose.rot.GetAsEuler( ).y,360));
			newmsg.values.push_back(fmod(pose.rot.GetAsEuler( ).z,360));
			newmsg.values.push_back(angular.x);
			newmsg.values.push_back(angular.y);
			newmsg.values.push_back(angular.z);
			newmsg.values.push_back(AccelLinear.x);
			newmsg.values.push_back(AccelLinear.y);
			newmsg.values.push_back(AccelLinear.z);
	
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
