/*
* File: QuadData
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 13/02/19
* Description:  This library is responsable to implement a sensor that return for a Quadrotor all data of the folowing state space:

- x,y,z,roll,pitch,yaw,dx,dy,dz,droll,dpitch,dyaw

*/


#include <QuadData.h>


namespace gazebo
{
	// constructor
	QuadData::QuadData(): PhipThetapPsip(3,1), RIB(3,3), W_n(3,3), WIIB(3,1), XpYpZp(3,1)
	{ 
				
	}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


	// destructor
	QuadData::~QuadData()
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


	// initial setup
	void QuadData::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "entrei Quad Estados plugin"<<std::endl;
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // Get name of topic to publish data
			
			
			world = _model->GetWorld();	// pointer to the world
			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf); // name of the main body
			link = _model->GetLink(link_name_); // pointer to the main body			

			// notifying when occurs new step time
	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&QuadData::Update, this));
			// ROS publisher
			publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);  //NameofNode é o nome do Topico na verdade
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


	// reset
	void QuadData::Reset()
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


	// new step time
	void QuadData::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock); // mutex
			
			simulator_msgs::Sensor newmsg;
			newmsg.name = NameOfNode_;
			newmsg.header.stamp = ros::Time::now(); // time stamp
			newmsg.header.frame_id = "1";
			ignition::math::Pose3d pose = link->WorldPose();
			newmsg.values.push_back(pose.Pos().X()); // x
			newmsg.values.push_back(pose.Pos().Y()); // y
			newmsg.values.push_back(pose.Pos().Z()); // z
			newmsg.values.push_back(pose.Rot().Euler( ).X()); //roll
			newmsg.values.push_back(pose.Rot().Euler( ).Y()); // pitch
			newmsg.values.push_back(pose.Rot().Euler( ).Z()); // yaw

			//compute the generalized velocities
			ignition::math::Vector3d linear = link->WorldLinearVel();
			ignition::math::Vector3d angular = link->WorldAngularVel( );
			
			Phi = pose.Rot().Euler( ).X();
			Theta = pose.Rot().Euler( ).Y();
			Psi = pose.Rot().Euler( ).Z();
			
			RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));
        	                
			W_n << 1.0,         0.0,          -sin(Theta), 
			       0.0,  cos(Phi),  cos(Theta)*sin(Phi),
	  	               0.0, -sin(Phi),  cos(Phi)*cos(Theta);
	  	               
			//Get the angular velocity w.r.t I expressed in I and maps to obtain the time derivative of Euler angles
			WIIB << angular.X(), angular.Y(), angular.Z();
			PhipThetapPsip = W_n.inverse() * (RIB.transpose() * WIIB);
			XpYpZp << linear.X(), linear.Y(), linear.Z();
			

			newmsg.values.push_back(linear.X()); // dx
			newmsg.values.push_back(linear.Y()); //dy
			newmsg.values.push_back(linear.Z()); //dz
			newmsg.values.push_back(PhipThetapPsip(0));
			newmsg.values.push_back(PhipThetapPsip(1));
			newmsg.values.push_back(PhipThetapPsip(2));
		
			std::cout << pose.Pos().X() << ";"<< pose.Pos().Y() << ";"<< pose.Pos().Z() << ";"<< pose.Rot().Euler( ).X() << ";" << pose.Rot().Euler( ).Y() << ";" << pose.Rot().Euler().Z() << ";" << std::endl;
		
			// publish data
			publisher_.publish(newmsg);					
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
			
			

			

	GZ_REGISTER_MODEL_PLUGIN(QuadData)
}
