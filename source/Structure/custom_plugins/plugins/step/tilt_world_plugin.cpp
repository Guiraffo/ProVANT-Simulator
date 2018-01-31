/*
* File: tilt_world_plugin.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement the synchronization system of the system

*/

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <gazebo/physics/World.hh>
#include "std_msgs/String.h"

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
	public:

	physics::WorldPtr objeto; // pointer to the world
	ros::Subscriber sub; // ROS subscriber
	ros::NodeHandle n; // ROS node handle

	// constructor
  	WorldPluginTutorial() : WorldPlugin()
  	{
  	}

	// for each step time	
	void chatterCallback(const std_msgs::String::ConstPtr& msg)
	{
 
		objeto->Step(1); // commando to new step time
	}

  	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  	{
                // the world starts paused  
		_world->SetPaused(true);                                                              
    		if (!ros::isInitialized())
    		{
      			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      			return;
    		}	

		// subscriber to receive signals of steptime
		sub = n.subscribe("Step", 1, &gazebo::WorldPluginTutorial::chatterCallback,this);
		objeto = _world; // save world's pointer
  	}
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
