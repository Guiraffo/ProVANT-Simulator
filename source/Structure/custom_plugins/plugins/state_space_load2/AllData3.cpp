#include <AllData3.h>


namespace gazebo
{
	AllData2::AllData2():distributionX(0,0),distributionY(0,0),distributionZ(0,0) //0.0025
	{ 
		
	}

	AllData2::~AllData2()
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

	void AllData2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfNode",_sdf);
			NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR",_sdf);
			NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL",_sdf);
                        NameOfJointLoad = XMLRead::ReadXMLString("NameOfJointLoad",_sdf);

			world = _model->GetWorld();	
			juntaR = _model->GetJoint(NameOfJointR_);
			juntaL = _model->GetJoint(NameOfJointL_);
                        juntaLoad = _model->GetJoint(NameOfJointLoad);

			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf);
			link = _model->GetLink(link_name_);			

	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&AllData2::Update, this));
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void AllData2::Reset()
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

	void AllData2::Update()
	{
		try
		{
			common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);
			
			math::Pose pose = link->GetWorldCoGPose();
			math::Vector3 linear = link->GetWorldCoGLinearVel();
			math::Vector3 angular = link->GetWorldAngularVel( );

			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(pose.pos.x);
			newmsg.values.push_back(pose.pos.y);
			newmsg.values.push_back(pose.pos.z);
			newmsg.values.push_back(pose.rot.GetAsEuler( ).x);
			newmsg.values.push_back(pose.rot.GetAsEuler( ).y);
			newmsg.values.push_back(pose.rot.GetAsEuler( ).z);
			newmsg.values.push_back(juntaR->GetAngle(0).Radian());
			newmsg.values.push_back(juntaL->GetAngle(0).Radian());
                        newmsg.values.push_back(juntaLoad->GetAngle(0).Radian());
                        newmsg.values.push_back(juntaLoad->GetAngle(1).Radian());
			newmsg.values.push_back(linear.x);
			newmsg.values.push_back(linear.y);
			newmsg.values.push_back(linear.z);
                        newmsg.values.push_back(angular.x);
                        newmsg.values.push_back(angular.y);
                        newmsg.values.push_back(angular.z);
			newmsg.values.push_back(juntaR->GetVelocity(0));
			newmsg.values.push_back(juntaL->GetVelocity(0));
                        newmsg.values.push_back(juntaLoad->GetVelocity(0));
                        newmsg.values.push_back(juntaLoad->GetVelocity(1));

			publisher_.publish(newmsg);
					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(AllData2)
}
