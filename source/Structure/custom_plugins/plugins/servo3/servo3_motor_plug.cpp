#include <servo3_motor_plug.h>

namespace gazebo
{
        void Servo3MotorPlugin::CallbackReferencias(std_msgs::Float64 msg)
	{
		try
		{	bool result = false;
			math::Vector3 input(0,msg.data,0);
			boost::mutex::scoped_lock scoped_lock(lock);
			if(Modo_ == "Torque")	link->AddRelativeTorque(input);
			if(Modo_ == "Posição")	result = junta->SetPosition(0,msg.data);
			//if(Modo_ == "Velocidade")  result = junta->SetPosition(0,msg.data);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	
	
	Servo3MotorPlugin::Servo3MotorPlugin()
	{ 
		torque = 0;
	}

	Servo3MotorPlugin::~Servo3MotorPlugin()
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

	void Servo3MotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				LOG4CXX_ERROR (loggerMyMain, "Nao inicializado!");
	      		        return;
	    		}

			NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint",_sdf);
			TopicSubscriber_ = XMLRead::ReadXMLString("TopicSubscriber",_sdf);
			TopicPublisher_ = XMLRead::ReadXMLString("TopicPublisher",_sdf);
			Modo_ = XMLRead::ReadXMLString("Modo",_sdf);
			Link_ = XMLRead::ReadXMLString("ChildLink",_sdf);
			
			world = _model->GetWorld();	
			junta = _model->GetJoint(NameOfJoint_);
			link = _model->GetLink(Link_);

			motor_subscriber_ = node_handle_.subscribe(TopicSubscriber_, 1, &gazebo::Servo3MotorPlugin::CallbackReferencias, this);
			motor_publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(TopicPublisher_, 5);

	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&Servo3MotorPlugin::Update, this));
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void Servo3MotorPlugin::Reset()
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

	void Servo3MotorPlugin::Update()
	{
		try
		{
			simulator_msgs::Sensor newmsg;
			newmsg.name = TopicPublisher_;
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(junta->GetAngle(0).Radian());
			newmsg.values.push_back(junta->GetVelocity(0));
			motor_publisher_.publish(newmsg);

		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	GZ_REGISTER_MODEL_PLUGIN(Servo3MotorPlugin)
}
