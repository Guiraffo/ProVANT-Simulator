#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include "XMLRead.h"
#include "std_msgs/Float64.h"


namespace gazebo
{
	class AnglePlugin : public ModelPlugin
	{
		// constructor
		public: AnglePlugin();
		// destructor 
  		public:virtual ~AnglePlugin();
		// initial setup 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset
  		public: virtual void Reset(); 
		// update fpr each step time 
  		protected: virtual void Update(); 
		// calback for receiving references
		
		physics::WorldPtr world; // world's pointer
		UpdateTimer updateTimer; // pointer for notifying new step time
		physics::JointPtr joint;
  		event::ConnectionPtr updateConnection; // connection pointer
		ros::Publisher publisher_; // ROS publisher	
  		ros::NodeHandle node_handle_; // ROS's node handle
	};
}
