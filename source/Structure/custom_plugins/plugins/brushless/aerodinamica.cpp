#include <aerodinamica.h>

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica::Aerodinamica()
	{
		
	}

	Aerodinamica::~Aerodinamica()
	{	
		
	}

	void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "Aerodinamica nao inicializado!" << std::endl;
	      		        return;
	    		}
			
			topic_FR = XMLRead::ReadXMLString("topic_FR",_sdf);
			topic_FL = XMLRead::ReadXMLString("topic_FL",_sdf);
			NameOfLinkDir_ = XMLRead::ReadXMLString("LinkDir",_sdf);
			NameOfLinkEsq_ = XMLRead::ReadXMLString("LinkEsq",_sdf);

			// capitular elementos da simulação
		
			linkR = _model->GetLink(NameOfLinkDir_);
			linkL = _model->GetLink(NameOfLinkEsq_);	

			// update timer
	  		Reset();

			// subscribers			
			motor_subscriberFR_ = node_handle_.subscribe(topic_FR, 1, &gazebo::Aerodinamica::CallbackFR, this);
			motor_subscriberFL_ = node_handle_.subscribe(topic_FL, 1, &gazebo::Aerodinamica::CallbackFL, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void Aerodinamica::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void Aerodinamica::CallbackFR(std_msgs::Float64 msg)
	{
		try
		{
			Fr = msg.data;
			math::Vector3 forceR(0,0,Fr);
			math::Vector3 torqueR(0,0,0.0178947368*Fr);
			linkR->AddRelativeForce(forceR);
			linkR->AddRelativeTorque(torqueR);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	void Aerodinamica::CallbackFL(std_msgs::Float64 msg)
	{
		try
		{	
			Fl = msg.data;
			math::Vector3 forceL(0,0,Fl);
			math::Vector3 torqueL(0,0,-0.0178947368*Fl);
			linkL->AddRelativeForce(forceL);
			linkL->AddRelativeTorque(torqueL);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	

	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}
