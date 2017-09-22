
#include <server.h>

namespace gazebo
{

	HilServer::HilServer() 
	{
		
	}

	HilServer::~HilServer()
	{	
		try
		{
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}

	void HilServer::Update()
	{
		Zoh();
	}

	void HilServer::thread()
	{
		  serial.read();
		  for (int i = 0; i < 5; ++i)
		  {
				std::cout << serial.input[i] << '\n';
		  }
	}

	void HilServer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "Load" << std::endl;
			// obtendo dados do arquivo de descrição "model.sdf"
			NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR",_sdf);
			NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL",_sdf);
			link_name_ = XMLRead::ReadXMLString("bodyName",_sdf);
			link_right_ = XMLRead::ReadXMLString("BrushlessR",_sdf);
			link_left_ = XMLRead::ReadXMLString("BrushlessL",_sdf);
			
			// apontando ponteiros para acesso a dados do mundo, elo e juntas
			world = _model->GetWorld();	
			link = _model->GetLink(link_name_);
			linkR = _model->GetLink(link_right_);
			linkL = _model->GetLink(link_left_);
			juntaR = _model->GetJoint(NameOfJointR_);
			juntaL = _model->GetJoint(NameOfJointL_);
			
			// Iniciando comunicação serial
			serial.functor = boost::bind(&HilServer::GetSerialData,this, _1,_2);
			serial.connect("/dev/ttyS10", 115200);
			serial.send("Configurando\r\n");
			
			//t = new boost::thread(boost::bind(&gazebo::HilServer::thread, this));
			
			// configurando temporizador para callback
			Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&HilServer::Update, this));
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void HilServer::Reset()
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

	// Método para receber dados do sistema embarcado (Callback)
	void HilServer::GetSerialData(const boost::system::error_code& e,std::size_t size)
	{
		if (!e)
		{
			std::istream is(&serial.buffer);
			std::string data(size,'\0');
			is.read(&data[0],size);
			std::cout<<"Received data:"<<data;
			// verificar tipo de solitação
			// se escrita
				//SetInputVANT20();
			// se leitura
				//GetStatesVANT20();
			SetSerialData();
		};

		//serial.startReceive();
	}
	
	// Método para enviar dados para o sistema embarcado 
	void HilServer::SetSerialData()
	{
			serial.send("Text\r\n");
	}
	// Método para Ler dados de simulação
	void HilServer::GetStatesVANT20()
	{
			// dados da pose inicial
			math::Pose pose = link->GetWorldPose();
			x = pose.pos.x; // x
			y = pose.pos.y; // y
			z = pose.pos.z; // z
			roll = pose.rot.GetAsEuler( ).x; // roll
			pitch = pose.rot.GetAsEuler( ).y; // pitch
			yaw = pose.rot.GetAsEuler( ).z; // yaw
			alphar = juntaR->GetAngle(0).Radian(); // alphaR
			alphar = juntaL->GetAngle(0).Radian(); // alphaL
			math::Vector3 linear = link->GetWorldLinearVel();
			vx = linear.x; // vx
			vy = linear.y; // vy
			vz = linear.z; // vz
			math::Vector3 angular = link->GetWorldAngularVel( );
			wx = angular.x; //wx
			wy = angular.y; //wy
			wz = angular.z; //wz
			dalphar = juntaR->GetVelocity(0); // dalphaR
			dalphal = juntaL->GetVelocity(0); // dalphaL	
	}
	// Método para escrever dados de simulação
	void HilServer::SetInputVANT20(double Fr_,double Fl_,double Tr_,double Tl_)
	{
			Fr = Fr_;
			Fl = Fl_;
			Tr = Tr_;
			Tl = Tl_;
	}
	void HilServer::Zoh()
	{
		// Força de propulsão do motor direito
			math::Vector3 forceR(0,0,Fr);
			math::Vector3 torqueR(0,0,0.0178947368*Fr);
			linkR->AddRelativeForce(forceR);
			linkR->AddRelativeTorque(torqueR);
			
			// Força de propulsão do motor esquerdo
			math::Vector3 forceL(0,0,Fl);
			math::Vector3 torqueL(0,0,-0.0178947368*Fl);
			linkL->AddRelativeForce(forceL);
			linkL->AddRelativeTorque(forceL);
	
			// Torque do servo direito
			juntaR->SetForce(0,Tr);
			// Torque do servo esquerdo
			juntaL->SetForce(0,Tl);
	}

	GZ_REGISTER_MODEL_PLUGIN(HilServer)
}
