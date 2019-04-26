
#include <get_angle_plugin.h>

namespace gazebo
{
	
	

	// constructor
	AnglePlugin::AnglePlugin()
	{ 

	}

	// destructor
	AnglePlugin::~AnglePlugin()
	{	
		
	}

	// initial setup
	void AnglePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		std::cout << "Hello World get angle plugin" << std::endl;
		
		world = _model->GetWorld(); // get World's pointer
		std::string NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint",_sdf); // Get name of the joint
		joint = _model->GetJoint(NameOfJoint_);
		std::string TopicPublisher_ = XMLRead::ReadXMLString("TopicPublisher",_sdf); // Get name of the joint
		
		
		publisher_ = node_handle_.advertise<std_msgs::Float64>(TopicPublisher_, 5);
		
		updateTimer.Load(world, _sdf);
	  	updateConnection = updateTimer.Connect(boost::bind(&AnglePlugin::Update, this)); //Codigo de update tirado da internet ...so aceita
	}
	
	// reset
	void AnglePlugin::Reset()
	{
		
	}

	// for each ste time
	void AnglePlugin::Update()
	{
		std::cout << "Hello World2 get angle plugin" << std::endl;
		double Angle = joint->GetAngle(0).Radian();
		std::cout <<"Angle: "<< Angle << std::endl;
		std_msgs::Float64 msg;
		msg.data = Angle;
		publisher_.publish(msg);
	}
	
	GZ_REGISTER_MODEL_PLUGIN(AnglePlugin)
}
