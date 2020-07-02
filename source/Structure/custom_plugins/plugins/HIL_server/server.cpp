
#include <server.h>
#include <time.h>

namespace gazebo
{

	HilServer::HilServer() 
	{
		Fr = 0;
		Fl = 0;
		Tr =0;
		Tl = 0;
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
		//static int i = 0;
		//mutex.lock();
		std::lock_guard<std::mutex> lck (mtx);
		SetInputVANT20(Fr,Fl,Tr,Tl);
		//std::cout << "Fr: " << Fr << std::endl;
		//std::cout << "Fl: " << Fl << std::endl;
		//std::cout << "Tr: " << Tr << std::endl;
		//std::cout << "Tl: " << Tl << std::endl;
		//mutex.unlock();
		//i++;
		//std::cout << "Contador: " << i << std::endl;
		
	}

	void HilServer::thread()
	{
		struct timespec start, stop;
		std::chrono::high_resolution_clock::time_point tf;
		std::chrono::high_resolution_clock::time_point tf2;
		std::chrono::high_resolution_clock::time_point to;
		std::chrono::high_resolution_clock::time_point to2;
		  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
		  to = std::chrono::high_resolution_clock::now();
		  to2 = to;
		  while(true)
		  {
			Frame frame;
			frame = receive(serial);
			
			if(frame.unbuild())
			{
				float flag = frame.getFloat();
				if(flag==1) // se 1
				{
					//std::cout << "1" << std::endl;

					//clock_gettime(CLOCK_REALTIME, &stop);
					/*double result = (stop.tv_sec - start.tv_sec) * 1e3 + (stop.tv_nsec - start.tv_nsec) / 1e6;    // in microseconds
					std::cout << result << std::endl;
					clock_gettime(CLOCK_REALTIME, &start);*/
				
					//depois
					tf = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double,std::nano> delta_t = tf-to;
					std::chrono::duration<double,std::nano> delta_t2 = tf-to2;
					//std::chrono::duration<double,std::ratio<1l,1000000l>> delta_t = 
					//std::chrono::duration_cast<std::chrono::miliseconds>(tf - to);
					std::cout << delta_t.count()/1000000.0 << "," << delta_t2.count()/1000000000.0 << ",";
				
					//std::cout << model->GetWorld()->GetRealTime().FormattedString() << ",";	
				
				
					//antes
					to = tf;
				
				
					GetStatesVANT20();
					SetSerialData();
				}
				else
				{
					if(flag==0) // se 0
					{    
						//std::cout << "0" << std::endl;
						//mutex.lock();
						std::lock_guard<std::mutex> lck (mtx);
						Fr = frame.getFloat();
						Fl = frame.getFloat();
						Tr = frame.getFloat();
						Tl = frame.getFloat();
						
						std::cout << Fr << ",";
						std::cout << Fl << ",";
						std::cout << Tr << ",";
						std::cout << Tl << std::endl;
						//mutex.unlock();
						
					}
					else
					{
						if(flag==3) // se 0
						{    
							std::lock_guard<std::mutex> lck (mtx);
							Fr = frame.getFloat();
							Fl = frame.getFloat();
							Tr = frame.getFloat();
							Tl = frame.getFloat();
							//Frame frame2;
							//frame2.addFloat(3);
							//frame2.build();
							//serial.send(frame2.buffer(),frame2.buffer_size());
							model->GetWorld()->SetPaused(false);
						}
						else
						{
							std::cout << "Deu ruim" << std::endl;
						}
					}
				}
				
				
			}
			
		  }
	}

	void HilServer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			model = _model;
		
			//std::cout << "Load" << std::endl;
		
			// conectando comunicação serial
			//if (!serial.connect("/tmp/ttyS1",115200)) exit(1);
			if (serial.connect("/dev/ttyUSB0",921600/*576000*/)) {
	
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
			
				t = new boost::thread(boost::bind(&gazebo::HilServer::thread, this));
			
				// configurando temporizador para callback
				Reset();
				updateTimer.Load(world, _sdf);
		  		updateConnection = updateTimer.Connect(boost::bind(&HilServer::Update, this));
	  		}
		
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

	// Método para enviar dados para o sistema embarcado 
	void HilServer::SetSerialData()
	{
			Frame frame;
			frame.addFloat(x);
			frame.addFloat(y);
			frame.addFloat(z);
			frame.addFloat(roll);
			frame.addFloat(pitch);
			frame.addFloat(yaw);
			frame.addFloat(alphar);
			frame.addFloat(alphal);
			frame.addFloat(vx);
			frame.addFloat(vy);
			frame.addFloat(vz);
			frame.addFloat(wx);
			frame.addFloat(wy);
			frame.addFloat(wz);
			frame.addFloat(dalphar);
			frame.addFloat(dalphal);
			frame.build();
			serial.send(frame.buffer(),frame.buffer_size());

			//std::cout << "Dados:" << std::endl;
			std::cout <<  x << ",";
			std::cout <<  y << ",";
			std::cout <<  z << ",";
			std::cout << roll << ",";
			std::cout << pitch << ",";
			std::cout << yaw << ",";
			std::cout << alphar << ",";
			std::cout << alphal << ",";
			std::cout << vx << ",";
			std::cout << vy << ",";
			std::cout << vz << ",";
			std::cout << wx << ",";
			std::cout << wy << ",";
			std::cout << wz << ",";
			std::cout << dalphar << ",";
			std::cout << dalphal << ",";

	}
	// Método para Ler dados de simulação
	void HilServer::GetStatesVANT20()
	{
			// dados da pose inicial
			ignition::math::Pose3d pose = link->WorldPose();
			x = pose.Pos().X(); // x
			y = pose.Pos().Y(); // y
			z = pose.Pos().Z(); // z
			roll = pose.Rot().Euler( ).X(); // roll
			pitch = pose.Rot().Euler( ).Y(); // pitch
			yaw = pose.Rot().Euler( ).Z(); // yaw
			alphar = juntaR->Position(0); // alphaR
			alphal = juntaL->Position(0); // alphaL
			ignition::math::Vector3d linear = link->WorldLinearVel();
			vx = linear.X(); // vx
			vy = linear.Y(); // vy
			vz = linear.Z(); // vz
			ignition::math::Vector3d angular = link->WorldAngularVel( );
			wx = angular.X(); //wx
			wy = angular.Y(); //wy
			wz = angular.Z(); //wz
			dalphar = juntaR->GetVelocity(0); // dalphaR
			dalphal = juntaL->GetVelocity(0); // dalphaL	
	}
	// Método para escrever dados de simulação
	void HilServer::SetInputVANT20(double Fr_,double Fl_,double Tr_,double Tl_)
	{
			
			// Força de propulsão do motor direito
			ignition::math::Vector3d forceR(0,0,Fr_);
			ignition::math::Vector3d torqueR(0,0,0.0178947368*Fr_);
			linkR->AddRelativeForce(forceR);
			linkR->AddRelativeTorque(torqueR);
			
			// Força de propulsão do motor esquerdo
			ignition::math::Vector3d forceL(0,0,Fl_);
			ignition::math::Vector3d torqueL(0,0,-0.0178947368*Fl_);
			linkL->AddRelativeForce(forceL);
			linkL->AddRelativeTorque(torqueL);
	
			// Torque do servo direito
			juntaR->SetForce(0,Tr_);
			// Torque do servo esquerdo
			juntaL->SetForce(0,Tl_);
	}

	GZ_REGISTER_MODEL_PLUGIN(HilServer)
}
