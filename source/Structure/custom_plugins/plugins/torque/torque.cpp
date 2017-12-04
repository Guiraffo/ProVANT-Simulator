#include <torque.h>

//using namespace gazebo::math;

namespace gazebo
{

	torque::torque()
	{
		
	}

	torque::~torque()
	{	
		
	}

	void torque::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "Aerodinamica nao inicializado!" << std::endl;
	      		        return;
	    		}
			
			topicX = XMLRead::ReadXMLString("topicX",_sdf);
			topicY = XMLRead::ReadXMLString("topicY",_sdf);
			topicZ = XMLRead::ReadXMLString("topicZ",_sdf);
			NameOfLink = XMLRead::ReadXMLString("Link",_sdf);

			// capitular elementos da simulação
			link = _model->GetLink(NameOfLink);	

			// update timer
	  		Reset();

			// subscribers			
			pertubation_subscriberX = node_handle_.subscribe(topicX, 1, &gazebo::torque::CallbackX, this);
			pertubation_subscriberY = node_handle_.subscribe(topicY, 1, &gazebo::torque::CallbackY, this);
			pertubation_subscriberZ = node_handle_.subscribe(topicZ, 1, &gazebo::torque::CallbackZ, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void torque::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void torque::CallbackX(std_msgs::Float64 msg)
	{
		try
		{
			Fx = msg.data;
			math::Vector3 force(Fx,0,0);
			link->AddRelativeTorque(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void torque::CallbackY(std_msgs::Float64 msg)
	{
		try
		{
			Fy = msg.data;
			math::Vector3 force(0,Fy,0);
			link->AddRelativeTorque(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void torque::CallbackZ(std_msgs::Float64 msg)
	{
		try
		{
			Fz = msg.data;
			math::Vector3 force(0,0,Fz);
			link->AddRelativeTorque(force);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(torque)
}
