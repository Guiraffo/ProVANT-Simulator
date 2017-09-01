
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include "std_msgs/Float64.h"
#include <gazebo/math/Vector3.hh>
#include <ros/package.h>
#include "XMLRead.h"




namespace gazebo
{

	class Server : public ModelPlugin
	{
		
		public: 
			Server(); 
	  		virtual ~Server(); 
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
	  	 	virtual void Reset();  

					
		protected: 
			//adicionada função callback para atualizar a força da fuselagem:
			virtual void Update(); 

		private: 
			
						
			boost::mutex lock;
			UpdateTimer updateTimer;
 			event::ConnectionPtr updateConnection;

						

	};
}

//#endif
