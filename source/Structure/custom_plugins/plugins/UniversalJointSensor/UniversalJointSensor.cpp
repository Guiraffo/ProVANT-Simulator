#include <UniversalJointSensor.h>


namespace gazebo
{
	UniversalJointSensor::UniversalJointSensor()
	{ 
				
	}

	UniversalJointSensor::~UniversalJointSensor()
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

	void UniversalJointSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf);
			NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint",_sdf);
			axis = XMLRead::ReadXMLString("Axis",_sdf);
			world = _model->GetWorld();	
			junta = _model->GetJoint(NameOfJoint_);
				
	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&UniversalJointSensor::Update, this));
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void UniversalJointSensor::Reset()
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

	void UniversalJointSensor::Update()
	{
		try
		{
			common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);
			
			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			int index;
			if(axis=="axis") index = 0;
			else
			{
				if(axis=="axis2") index = 1;
				else 
				{	
					std::cout << "Erro no indice da junta, em UniversalJointSensor";
					exit(1);
				}
			}
			newmsg.values.push_back(junta->GetAngle(index).Radian());
			newmsg.values.push_back(junta->GetVelocity(index));
			newmsg.values.push_back(junta->GetForce(index));
			
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(UniversalJointSensor)
}
