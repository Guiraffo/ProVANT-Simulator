
#include <server.h>

namespace gazebo
{

	Server::Server() 
	{
		
	}

	Server::~Server()
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

	void Server::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	  		if (!ros::isInitialized())
	  		{
	    			std::cout << "Server nao inicializado!" << std::endl;
	    		        return;
	  		}
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void Server::Reset()
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

	void Server::Update()
	{
		try
		{	
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	

	GZ_REGISTER_MODEL_PLUGIN(Server)
}
