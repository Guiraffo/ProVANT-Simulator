#include <aerodinamica2.h>

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica2::Aerodinamica2()//:SaveDataWind(6,1)
	{
		rho = 1.21;	//densidade do ar
		sf = 0.0146; //area da superficie da fuselagem
		sh = 0.01437; //area da superficie do estabilizador horizontal
		sv = 0.013635; //area da superficie do estabilizador vertical
		
		R_IB.setZero(3,3);
		wind_B.setZero(3,1); //vento no corpo
		wind_I.setZero(3,1); //vento inercial
        T = 0.0001;
		
		std::string relativeFile("Wind.txt");
		std::string file = std::getenv("TILT_MATLAB") + relativeFile;

		Wind.startFile(file,"wind");
	}

	Aerodinamica2::~Aerodinamica2()
	{	Wind.endFile();
		try
		{
			updateTimer.Disconnect(updateConnection);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}

	void Aerodinamica2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	  		if (!ros::isInitialized())
	  		{
	    			std::cout << "Aerodinamica2 nao inicializado!" << std::endl;
	    		        return;
	  		}
			
			topic_Elev = XMLRead::ReadXMLString("topic_Elev",_sdf);
			topic_Rud = XMLRead::ReadXMLString("topic_Rud",_sdf);
			topic_Fr = XMLRead::ReadXMLString("topic_Fr",_sdf);
			topic_Fl = XMLRead::ReadXMLString("topic_Fl",_sdf);
			
			NameOfLinkBody_ = XMLRead::ReadXMLString("bodyName",_sdf);
			NameOfLinkFr_ = XMLRead::ReadXMLString("LinkFr",_sdf);
			NameOfLinkFl_ = XMLRead::ReadXMLString("LinkFl",_sdf);
			NameOfLinkElev_ = XMLRead::ReadXMLString("LinkElev",_sdf);			
			NameOfLinkRud_ = XMLRead::ReadXMLString("LinkRud",_sdf);
			NameOfJointFr_ = XMLRead::ReadXMLString("JointFr",_sdf);			
			NameOfJointFl_ = XMLRead::ReadXMLString("JointFl",_sdf);
		
			world = _model->GetWorld();	
			
			linkFr = _model->GetLink(NameOfLinkFr_);
			linkFl = _model->GetLink(NameOfLinkFl_);
			linkE = _model->GetLink(NameOfLinkElev_);
			linkR = _model->GetLink(NameOfLinkRud_);			
			link = _model->GetLink(NameOfLinkBody_);		
			
			JointFr = _model->GetJoint(NameOfJointFr_);
			JointFl = _model->GetJoint(NameOfJointFl_);	
			
			// update timer
		  	Reset();
		  	updateTimer.Load(world, _sdf);
			updateConnection = updateTimer.Connect(boost::bind(&Aerodinamica2::Update, this));

			// subscribers			
			motor_subscriberFR_ = node_handle_.subscribe(topic_Fr, 1, &gazebo::Aerodinamica2::CallbackFR, this);
			motor_subscriberFL_ = node_handle_.subscribe(topic_Fl, 1, &gazebo::Aerodinamica2::CallbackFL, this);
			motor_subscriberFElev_ = node_handle_.subscribe(topic_Elev, 1, &gazebo::Aerodinamica2::CallbackFElev, this);
			motor_subscriberFRud_ = node_handle_.subscribe(topic_Rud, 1, &gazebo::Aerodinamica2::CallbackFRud, this);
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void Aerodinamica2::Reset()
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

	void Aerodinamica2::CallbackFR(std_msgs::Float64 msg)
	{
		try
		{
			math::Vector3 forceR(0,0,msg.data);
			linkFr->AddRelativeForce(forceR);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	void Aerodinamica2::CallbackFL(std_msgs::Float64 msg)
	{
		try
		{	
			math::Vector3 forceL(0,0,msg.data);
			linkFl->AddRelativeForce(forceL);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	
	void Aerodinamica2::CallbackFElev(std_msgs::Float64 De)
	{
		try
		{
			ElevatorDeflection = De.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	

	void Aerodinamica2::CallbackFRud(std_msgs::Float64 Dr)
	{
		try
		{
			RudderDeflection = Dr.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	

	void Aerodinamica2::Update()
	{
		try
		{	
			//Fuselagem
			common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);

			//Calculo do vento
			math::Vector3 linear = link->GetWorldLinearVel();				
			double xp = linear.x; 
			double yp = linear.y;
			double zp = linear.z; 

			//std::cout << "linear: " << xp << yp << zp << std::endl;
			math::Pose pose = link->GetWorldPose();			
			phi = pose.rot.GetAsEuler().x;
			theta = pose.rot.GetAsEuler().y;
			psi = pose.rot.GetAsEuler().z;	
			
			//std::cout << "euler: " << phi << theta << psi << std::endl;
			
			R_IB << (cos(psi)*cos(theta)), (cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi)), (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)),
				(cos(theta)*sin(psi)), (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)),
				(-sin(theta)), (cos(theta)*sin(phi)), (cos(phi)*cos(theta));
	
			Eigen::MatrixXd XpYpZp(3,1);
			Eigen::MatrixXd uvw(3,1);
			
			XpYpZp << xp, yp, zp;
			uvw = R_IB.transpose()*XpYpZp;
			
			ub = uvw(0);
			vb = uvw(1);
			wb = uvw(2);
			static double Tempo = 0;
			Tempo = Tempo + T;
		  	// Environment wind
				wind_I << 3, 0, 0;
			
				wind_B = R_IB.transpose() * wind_I;
				ua = wind_B(0,0);
				va = wind_B(1,0);
				wa = wind_B(2,0);
			std::vector<double> DataWind;
			DataWind.push_back(ua);
			DataWind.push_back(va);
			DataWind.push_back(wa);
			DataWind.push_back(wind_I(0));
			DataWind.push_back(wind_I(1));
			DataWind.push_back(wind_I(2));
			static int Contador = 0;
			if(Contador%120 == 0){
			Wind.printFile(DataWind);
			}
//			SaveDataWind << ua, va, wa, wind_I(0),  wind_I(1),  wind_I(2);
//			std::vector<double> out(SaveDataWind.data(), SaveDataWind.data() + SaveDataWind.rows() * SaveDataWind.cols());
			alpha = getAlpha(wb,wa,ub,ua);
			beta = getBeta(vb,va,ub,ua);	
			air_xy = getAirxy(vb,va,ub,ua);
			air_xz = getAirxz(wb,wa,ub,ua);
			v_xy = pow(air_xy,2);
			v_xz = pow(air_xz,2);

			double Val = ((alpha+3.1416)/0.1)+1.0;
			int IndexA = floor(Val);
			double ProporcaoA = Val - IndexA;

			Val = ((beta+3.1416)/0.1)+1.0;
			int IndexS = floor(Val);
			double ProporcaoS = Val - IndexS;

			double CDfa = VetCDf[IndexA] + ProporcaoA*(VetCDf[IndexA+1] - VetCDf[IndexA]);
			double CLfa = VetCLf[IndexA] + ProporcaoA*(VetCLf[IndexA+1] - VetCLf[IndexA]);
			double CDfs = VetCDf[IndexS] + ProporcaoS*(VetCDf[IndexS+1] - VetCDf[IndexS]);
			double CLfs = VetCLf[IndexS] + ProporcaoS*(VetCLf[IndexS+1] - VetCLf[IndexS]);
			double CDh  = VetCDh[IndexA] + ProporcaoA*(VetCDh[IndexA+1] - VetCDh[IndexA]);
			double CLh  = VetCLh[IndexA] + ProporcaoA*(VetCLh[IndexA+1] - VetCLh[IndexA]);
			double CDv  = VetCDv[IndexS] + ProporcaoS*(VetCDv[IndexS+1] - VetCDv[IndexS]);
			double CLv  = VetCLv[IndexS] + ProporcaoS*(VetCLv[IndexS+1] - VetCLv[IndexS]);


			//Forças aerodinâmicas aplicadas à fuzelagem
			Eigen::MatrixXd Ff_B(3,1), Ff_I(3,1);
			Ff_B(0) = (0.5)*rho*sf*v_xz*( CLfa*sin(alpha) - CDfa*cos(alpha) ) +
				   (0.5)*rho*sf*v_xy*( CLfs*sin(beta)  - CDfs*cos(beta) );
			Ff_B(1) = (0.5)*rho*sf*v_xy*( CLfs*cos(beta)  + CDfs*sin(beta) );
			Ff_B(2) = (0.5)*rho*sf*v_xz*( CLfa*cos(alpha) + CDfa*sin(alpha));
			//std::cout << "Alpha: " << alpha << std::endl;
			//std::cout << "Beta: " << beta << std::endl;
			//link->AddRelativeForce(Ff);
			Ff_I = R_IB*Ff_B; // Expressa forças no inercial
			Ff.x = Ff_I(0);
			Ff.y = Ff_I(1);
			Ff.z = Ff_I(2);
			link->AddForce(Ff); // AddForce -> Aplica forças com relação ao referencial inercial
			//std::cout << "Fuzelagem" << "Forças: " << Ff << std::endl;
			
			//Rudder		
			Eigen::MatrixXd Fv_B(3,1), Fv_I(3,1);
			Fv_B(0) = (0.5)*rho*sv*v_xy*( ( CLv + c_r(RudderDeflection) )*sin(beta) - CDv*cos(beta) );
			Fv_B(1) = (0.5)*rho*sv*v_xy*( CDv*sin(beta) + ( CLv + c_r(RudderDeflection) )*cos(beta) );
			Fv_B(2) = 0.0;
			Fv_I = R_IB*Fv_B; // Expressa forças no inercial
			Fv.x = Fv_I(0);
			Fv.y = Fv_I(1);
			Fv.z = Fv_I(2);

			linkR->AddForce(Fv);	
			//std::cout << "Rudder def: " << RudderDeflection << "Forças: " << Fv << std::endl;
			
			//Elevator

			Eigen::MatrixXd Fh_B(3,1), Fh_I(3,1);
			Fh_B(0) = (0.5)*rho*sh*v_xz*( ( CLh + c_e(ElevatorDeflection) )*sin(alpha) - CDh*cos(alpha) );
			Fh_B(1) = 0.0;
			Fh_B(2) = (0.5)*rho*sh*v_xz*( CDh*sin(alpha) + ( CLh + c_e(ElevatorDeflection) )*cos(alpha) );
			Fh_I = R_IB*Fh_B; // Expressa forças no inercial
			Fh.x = Fh_I(0);
			Fh.y = Fh_I(1);
			Fh.z = Fh_I(2);
			linkE->AddForce(Fh);		
			//std::cout << "Elevator def: " << ElevatorDeflection << "Forças: " << Fh << std::endl;


		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	

	double Aerodinamica2::getAlpha(double wb, double wa, double ub, double ua)
	{
		try
		{
			double alpha;
			alpha = -atan2((wb-wa),(ub-ua));
			if(alpha > 3.0584)
			{
				alpha = 3.0584;
			}else if( alpha < -3.1416){
				alpha = -3.1416;
			}
			return alpha;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}


	double Aerodinamica2::getBeta(double vb, double va, double ub, double ua)
	{
		try
		{
			double beta;
			beta = -atan2((vb-va),(ub-ua));
			if(beta > 3.0584)
			{
				beta = 3.0584;
			}else if( beta < -3.1416){
				beta = -3.1416;
			}
			return beta;
		}
		catch(std::exception& e)	
		{
			std::cout << e.what() << std::endl;
		}
	}

	double Aerodinamica2::getAirxz(double wb, double wa, double ub, double ua)
	{
		try
		{
			return sqrt(pow(wb-wa,2) + pow(ub-ua,2));
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}

	double Aerodinamica2::getAirxy(double vb, double va, double ub, double ua)
	{
		try
		{
			return sqrt(pow(vb-va,2) + pow(ub-ua,2));
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}

//	double Aerodinamica2::cd_fxz(double alpha)
//	{
//		try
//		{
//	
//			return 0.4566*pow(alpha,2)-0.0403*alpha+0.0601;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	

//	}
//	
//	double Aerodinamica2::cl_fxz(double alpha)
//	{
//		try
//		{
//			return 0.5405*alpha-0.0353;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}

//	double Aerodinamica2::cd_fxy(double beta)
//	{
//		try
//		{
//			return 0.3513*pow(beta,2) + 0.0604;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}

//	double Aerodinamica2::cl_fxy(double beta)
//	{
//		try
//		{
//			return 0.3821*beta;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}

//	double Aerodinamica2::cd_vxy(double beta)
//	{
//		try
//		{

//			return 2.2019*pow(beta,2) + 0.0149;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}

//	double Aerodinamica2::cl_vxy(double beta)
//	{
//		try
//		{
//			return -45.392*pow(beta,3) + 0.0011*pow(beta,2) + 6.1126*beta;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}	
//	
//	double Aerodinamica2::cd_hxz(double alpha)
//	{
//		try
//		{
//			return 1.9382*pow(alpha,2)+0.0088;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}
//	
//	double Aerodinamica2::cl_hxz(double alpha)
//	{
//		try
//		{
//			return -35.216*pow(alpha,3) + 6.5360*alpha;
//		}
//		catch(std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//		} 	
//	}

	double Aerodinamica2::c_r(double Dr)
	{
		try
		{
			return 1.165*Dr;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}

	double Aerodinamica2::c_e(double De)
	{
		try
		{
			return 2.1873375*De;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica2)
}
