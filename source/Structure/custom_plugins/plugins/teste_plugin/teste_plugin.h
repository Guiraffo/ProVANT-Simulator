
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>



namespace gazebo
{
	class testePlugin : public ModelPlugin
	{
		// constructor
		public: testePlugin();
		// destructor 
  		public:virtual ~testePlugin();
		// initial setup 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset
  		public: virtual void Reset(); 
		// update fpr each step time 
  		protected: virtual void Update(); 
		// calback for receiving references
		
		physics::WorldPtr world; // world's pointer
		UpdateTimer updateTimer; // pointer for notifying new step time
			
  		event::ConnectionPtr updateConnection; // connection pointer
			

	};
}
