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

	physics::WorldPtr objeto;
	ros::Subscriber sub;
	ros::NodeHandle n;

  	WorldPluginTutorial() : WorldPlugin()
  	{
  	}

	void chatterCallback(const std_msgs::String::ConstPtr& msg)
	{
 
		objeto->Step(1);
	}

  	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  	{
                  

		_world->SetPaused(true);                                                              
    		if (!ros::isInitialized())
    		{
      			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      			return;
    		}	

		//ros::init(argc, argv, "Teste");
		sub = n.subscribe("Step", 1, &gazebo::WorldPluginTutorial::chatterCallback,this);

		
		objeto = _world;
		
  	}



};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
