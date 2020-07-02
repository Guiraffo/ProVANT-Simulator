#include <servo2_motor_plug.h>

namespace gazebo
{
  void Servo2MotorPlugin::CallbackReferencias(std_msgs::Float64 msg)
	{
		try
		{	
			boost::mutex::scoped_lock scoped_lock(lock);
			
			if(Modo_ == "Torque") 
			{ 
				junta->SetForce(0,msg.data); 
			}
			else
			{
	
				this->model->GetJointController()->SetPositionTarget(this->junta->GetScopedName(), msg.data);  				
			}
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	Servo2MotorPlugin::Servo2MotorPlugin()
	{ 
		torque = 0;
	}

	Servo2MotorPlugin::~Servo2MotorPlugin()
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

	void Servo2MotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
			
			world = _model->GetWorld();	
			junta = _model->GetJoint(NameOfJoint_);



			// -------------------------------------------------------------------------

			this->model = _model;

			if(Modo_ == "Posicao")
			{
		    	// Setup de um controlador proporcional (P) com um ganho de 0.1:
//  				this->pid = common::PID(0.010, 0.050, 0.0001);

  				if(NameOfJoint_ == "elev"){
  					this->pid = common::PID(0.010, 0.050, 0.0001);
  				}
  				else{
  					this->pid = common::PID(0.001, 0.0050, 0.00001);
  				}
  				
  				// Aplica o controlador na junta:
  				this->model->GetJointController()->SetPositionPID(this->junta->GetScopedName(),this->pid);
			}

			// -------------------------------------------------------------------------



			motor_subscriber_ = node_handle_.subscribe(TopicSubscriber_, 1, &gazebo::Servo2MotorPlugin::CallbackReferencias, this);
			motor_publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(TopicPublisher_, 5);

	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&Servo2MotorPlugin::Update, this));
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void Servo2MotorPlugin::Reset()
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

	void Servo2MotorPlugin::Update()
	{
		try
		{
			simulator_msgs::Sensor newmsg;
			newmsg.name = TopicPublisher_;
			newmsg.header.stamp = ros::Time::now();
			newmsg.header.frame_id = "1";
			newmsg.values.push_back(junta->Position(0));
			newmsg.values.push_back(junta->GetVelocity(0));
			motor_publisher_.publish(newmsg);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	GZ_REGISTER_MODEL_PLUGIN(Servo2MotorPlugin)
}
