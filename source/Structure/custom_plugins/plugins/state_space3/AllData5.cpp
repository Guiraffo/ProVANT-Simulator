#include <AllData5.h>
 
namespace gazebo
{
	AllData5::AllData5():distributionX(0,0),distributionY(0,0),distributionZ(0,0) 
	{ 
				
	}

	AllData5::~AllData5()
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

	void AllData5::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
			
			world = _model->GetWorld();	
			juntaR = _model->GetJoint(NameOfJointR_);
			juntaL = _model->GetJoint(NameOfJointL_);		

			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf);
			link = _model->GetLink(link_name_);			

			Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&AllData5::Update, this));
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);

		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void AllData5::Reset()
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

	void AllData5::Update()
	{
		try
		{
			common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);
//			data.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);

			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;	
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			math::Pose pose = link->GetWorldPose();
			
			newmsg.values.push_back(pose.pos.x);
			newmsg.values.push_back(pose.pos.y);
			newmsg.values.push_back(pose.pos.z);
			newmsg.values.push_back(pose.rot.GetAsEuler().x);
			newmsg.values.push_back(pose.rot.GetAsEuler().y);
			newmsg.values.push_back(pose.rot.GetAsEuler().z);			
			newmsg.values.push_back(juntaR->GetAngle(0).Radian());
			newmsg.values.push_back(juntaL->GetAngle(0).Radian());

			math::Vector3 linear = link->GetWorldLinearVel();
		
			newmsg.values.push_back(linear.x);
			newmsg.values.push_back(linear.y);
			newmsg.values.push_back(linear.z);
		
			math::Vector3 angular = link->GetWorldAngularVel();

			newmsg.values.push_back(angular.x*(1) + angular.y*((sin(pose.rot.GetAsEuler( ).x)*sin(pose.rot.GetAsEuler( ).y))/cos(pose.rot.GetAsEuler( ).y)) + angular.z*((cos(pose.rot.GetAsEuler( ).x)*sin(pose.rot.GetAsEuler( ).y))/cos(pose.rot.GetAsEuler( ).y)));

			
			newmsg.values.push_back(angular.x*(0) + angular.y*(cos(pose.rot.GetAsEuler( ).x)) + angular.z*(-sin(pose.rot.GetAsEuler( ).x)));
			
			newmsg.values.push_back(angular.x*(0) + angular.y*(sin(pose.rot.GetAsEuler( ).x)/cos(pose.rot.GetAsEuler( ).y)) + angular.z*(cos(pose.rot.GetAsEuler( ).x)/cos(pose.rot.GetAsEuler( ).y)));

			
			//math::Vector3 linear = link->GetRelativeLinearVel();				
			//newmsg.values.push_back(linear.x); 
			//newmsg.values.push_back(linear.y);
			//newmsg.values.push_back(linear.z); 
			//math::Vector3 angular = link->GetRelativeAngularVel();	
			//newmsg.values.push_back(angular.x); 
			//newmsg.values.push_back(angular.y); 
			//newmsg.values.push_back(angular.z);	
								
			newmsg.values.push_back(juntaR->GetVelocity(0)); 
			newmsg.values.push_back(juntaL->GetVelocity(0)); 
			
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	GZ_REGISTER_MODEL_PLUGIN(AllData5)
}
