/*
* File: PathPlotter.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 14/10/19
* Description:  This library is responsable to allow visualize the uav, the uav trajectory and reference trajectory in rviz

*/


#include <PathPlotter.h>
#include <eigen3/Eigen/Eigen>

namespace gazebo
{
	// constructor
	PathPlotter::PathPlotter()
	{ 
			
	}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


	// destructor
	PathPlotter::~PathPlotter()
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


	// initial setup
	void PathPlotter::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "entrei";
	    		if (!ros::isInitialized())
	    		{
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			
			NameOftopic_path_ = XMLRead::ReadXMLString("NameOfPathTopic",_sdf); // Get name of topic to publish data
			NameOftopic_mark_ = XMLRead::ReadXMLString("NameOfMarkerTopic",_sdf); // Get name of topic to publish data
			NameOftopic_ref_ = XMLRead::ReadXMLString("NameOfPathRefTopic",_sdf); // Get name of topic to publish data
			tag_ = XMLRead::ReadXMLString("uav",_sdf);	//tag to identify which uav we're using
			
			world = _model->GetWorld();	// pointer to the world
			

			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf); // name of the main body
			link = _model->GetLink(link_name_); // pointer to the main body			

			// notifying when occurs new step time
	  	Reset();
			updateTimer.Load(world, _sdf);
	  	updateConnection = updateTimer.Connect(boost::bind(&PathPlotter::Update, this));
			// ROS publisher
			publisher_path_ = node_handle_.advertise<nav_msgs::Path>(NameOftopic_path_, 1);
			publisher_marker_ = node_handle_.advertise<visualization_msgs::Marker>(NameOftopic_mark_, 1);
			publisher_path_ref_ = node_handle_.advertise<nav_msgs::Path>(NameOftopic_ref_, 1);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


	// reset
	void PathPlotter::Reset()
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


	// new step time
	void PathPlotter::Update()
	{
		try
		{
			boost::mutex::scoped_lock scoped_lock(lock); // mutex
			math::Pose pose = link->GetWorldPose();
			ros::Time time = ros::Time::now();
			double tempo =time.toSec();

		
		//broadcast for the reference trajectory
			tf::TransformBroadcaster broadcaster_ref;
			if(tag_ == "vant_4_aerod"){
			broadcaster_ref.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0,0,0, 1), tf::Vector3(4*sin(tempo/2),4*cos(tempo/2),tempo/10)),
      ros::Time::now(),"inertial_frame", "base_link"));
			}else if(tag_ == "quadcopter"){broadcaster_ref.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0,0,0, 1), tf::Vector3(6*sin(tempo/2),6*cos(tempo/2),tempo/3)),
      ros::Time::now(),"inertial_frame", "base_link"));
      }
      
			nav_msgs::Path path_ref;
			if(tag_ == "quadcopter"){
			geometry_msgs::PoseStamped pose_path_ref;
			pose_path_ref.header.stamp = ros::Time::now();
			pose_path_ref.header.frame_id = "inertial_frame";
			pose_path_ref.pose.position.x = sin(tempo/2);
			pose_path_ref.pose.position.y = cos(tempo/2);
			pose_path_ref.pose.position.z = tempo/10;
			pose_path_ref.pose.orientation.x = 0;
			pose_path_ref.pose.orientation.y = 0;
			pose_path_ref.pose.orientation.z = 0;
			pose_path_ref.pose.orientation.w = 1;
			path_ref.header.stamp= ros::Time::now();
			path_ref.header.frame_id="inertial_frame";
			path_ref.poses.push_back(pose_path_ref);
			//Publishes the reference trajectory 
			publisher_path_ref_.publish(path_ref);				
			}else if(tag_ == "vant_4_aerod") 		{geometry_msgs::PoseStamped pose_path_ref;
			pose_path_ref.header.stamp = ros::Time::now();
			pose_path_ref.header.frame_id = "inertial_frame";
			pose_path_ref.pose.position.x = 6*sin(tempo/2);
			pose_path_ref.pose.position.y = 6*cos(tempo/2);
			pose_path_ref.pose.position.z = tempo/3;
			pose_path_ref.pose.orientation.x = 0;
			pose_path_ref.pose.orientation.y = 0;
			pose_path_ref.pose.orientation.z = 0;
			pose_path_ref.pose.orientation.w = 1;
			path_ref.header.stamp= ros::Time::now();
			path_ref.header.frame_id="inertial_frame";
			path_ref.poses.push_back(pose_path_ref);
			//Publishes the reference trajectory 
			publisher_path_ref_.publish(path_ref);	
			}
			
						
			
			//broadcaster for the real trajectory
      tf::TransformBroadcaster broadcaster;
			broadcaster.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(pose.rot.GetAsEuler().x,pose.rot.GetAsEuler().y,pose.rot.GetAsEuler().z, 1), tf::Vector3(pose.pos.x,pose.pos.y,pose.pos.z)),
      ros::Time::now(),"inertial_frame", "base_link"));
       
       
			nav_msgs::Path path;
		
			geometry_msgs::PoseStamped pose_path;
			pose_path.header.stamp = ros::Time::now();
			pose_path.header.frame_id = "inertial_frame";
			pose_path.pose.position.x = pose.pos.x;
			pose_path.pose.position.y = pose.pos.y;
			pose_path.pose.position.z = pose.pos.z;
			pose_path.pose.orientation.x = pose.rot.GetAsEuler().x;
			pose_path.pose.orientation.y = pose.rot.GetAsEuler().y;
			pose_path.pose.orientation.z = pose.rot.GetAsEuler().z;
			pose_path.pose.orientation.w = 1;			
			
			path.header.stamp= ros::Time::now();
			path.header.frame_id="inertial_frame";
			path.poses.push_back(pose_path);
			
			publisher_path_.publish(path);	// Publishes the real trajectory
			
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
			
			//Marker for visualization
			
			visualization_msgs::Marker marker;
			marker.header.stamp = ros::Time::now();
			marker.header.frame_id = "base_link";
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = marker.MESH_RESOURCE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = 0;
			marker.pose.position.y = 0;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;
			marker.color.r = 0;
			marker.color.g = 0;
			marker.color.b = 1;
			marker.color.a = 1;	
			if(tag_ == "vant_4_aerod"){
			marker.mesh_resource = "package://Database/models/vant_4_aerod/meshes/mainbodyrviz.dae";
			}else if(tag_ == "quadcopter"){
			marker.mesh_resource = "package://Database/models/quadcopter/meshes/main_body.dae";
			}
			publisher_marker_.publish(marker);	
				
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(PathPlotter)
}
