/*
* File: PathPlotter.h
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 14/10/19
* Description:  This library is responsable to allow visualize the uav trajectory and reference trajectory in rviz

*/

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include <iostream>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <random>
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>



namespace gazebo
{
	class PathPlotter : public ModelPlugin
	{
		// constructor
		public: PathPlotter();
		// destructor 
  		public:virtual ~PathPlotter(); 
		// initial setup
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		// reset 
  		public: virtual void Reset();
		// for each steo time  
  		protected: virtual void Update(); 
		
		private:  
		
			std::string NameOftopic_path_;
			std::string NameOftopic_mark_;
			std::string NameOftopic_ref_;
			std::string link_name_; // name of link
			std::string tag_; //To identify which uav we're using
			physics::LinkPtr link; // pointer to the link
			physics::WorldPtr world; // pointer to the world
			UpdateTimer updateTimer;  // update time
  			event::ConnectionPtr updateConnection; // update connection
			ros::NodeHandle node_handle_; // ROS's node handle
			boost::mutex lock; // mutex
			ros::Publisher publisher_path_;  // ROS publisher
			ros::Publisher publisher_path_ref_;		
			ros::Publisher publisher_marker_;	
			tf::TransformBroadcaster broadcaster;
			tf2_ros::StaticTransformBroadcaster broadcaster_2;
	};
}
