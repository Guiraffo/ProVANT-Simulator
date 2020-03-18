#include <aerodinamica2.h>

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica2::Aerodinamica2():DBf(3),DBh(3),DBv(3)//:SaveDataWind(6,1)
	{
		Ho = 1.21;	//densidade do ar
		sf = 0.0146; //area da superficie da fuselagem
		sh = 0.01437; //area da superficie do estabilizador horizontal
		sv = 0.013635; //area da superficie do estabilizador vertical
		
		R_IB.setZero(3,3);
		wind_B.setZero(3,1); //vento no corpo
		wind_I.setZero(3,1); //vento inercial
        	//T = 0.0001;
        	T = 0.001;
		
		std::string relativeFile("Wind.txt");
		std::string file = std::getenv("TILT_MATLAB") + relativeFile;

		Wind.startFile(file,"wind");
		
		
		//Position of the Aerodinamic centers w.r.t the Body frame expressed in Body frame
		Eigen::VectorXd PosCG(3);
		
		PosCG << 0.008, 0, -0.043;
		DBf  << -0.005, 0, 0.0326; // Position aerodynamic center of fuselage
		DBh << -0.375547, 0, 0.030084; // Position aerodynamic center of vertical stabilizer
		DBv << -0.42236, 0, 0.11569; // Position aerodynamic center of horizontal stabilizer
		
		
		DBf  =  DBf - PosCG;
		DBh = DBh - PosCG;	
		DBv = DBv - PosCG;	
		
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
		
			world = _model->GetWorld();	
			
			linkFr = _model->GetLink(NameOfLinkFr_);
			linkFl = _model->GetLink(NameOfLinkFl_);			
			link = _model->GetLink(NameOfLinkBody_);			
			
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
			math::Vector3 torqueR(0,0,0.0178947368*msg.data); // drag torque
			linkFr->AddRelativeForce(forceR);
			linkFr->AddRelativeTorque(torqueR);
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
			math::Vector3 torqueL(0,0,-0.0178947368*msg.data); // drag torque
			linkFl->AddRelativeForce(forceL);
			linkFl->AddRelativeTorque(torqueL);
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

			math::Pose pose = link->GetWorldPose();			
			phi = pose.rot.GetAsEuler().x;
			theta = pose.rot.GetAsEuler().y;
			psi = pose.rot.GetAsEuler().z;	
			
			
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
			
		  	// Environment wind properties
			wind_I << 0, 0, 0;
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
//			std::cout << std::endl << "ub: " << ub << "ua: " << ua << "vb: " << vb << "va: " << va << "wb: " << wb << "wa: " << wa << std::endl;
			
			Alpha = getAlpha(wb,wa,ub,ua);
			Beta = getBeta(vb,va,ub,ua);	
			
			Eigen::MatrixXd RBAlpha(3,3), RBBeta(3,3);
			RBAlpha <<   cos(-Alpha),  0,   sin(-Alpha),
                       			       0,  1,             0,
            		  	    -sin(-Alpha),  0,   cos(-Alpha);

			RBBeta <<    cos(-Beta),  -sin(-Beta), 0,
				     sin(-Beta),   cos(-Beta), 0,
					      0,            0, 1;
					      
//			std::cout << "xp: " << xp << std::endl;
//			std::cout << "yp: " << yp << std::endl;
//			std::cout << "zp: " << zp << std::endl;
//			std::cout << "ub: " << ub << std::endl;
//			std::cout << "Alpha: " << Alpha << std::endl;
//			std::cout << "Beta: " << Beta << std::endl;
//			std::cout << "Phi: " << phi << std::endl;
//			std::cout << "Theta: " << theta << std::endl;
//			std::cout << "Psi: " << psi << std::endl;
			Vxy = getAirxy(vb,va,ub,ua);
			Vxz = getAirxz(wb,wa,ub,ua);

			double Val = ((-Alpha+3.1416)/0.1);
			int IndexA = floor(Val);
			double ProporcaoA = Val - IndexA;

			Val = ((-Beta+3.1416)/0.1);
			int IndexS = floor(Val);
			double ProporcaoS = Val - IndexS;

			double CDfxz = VetCDf[IndexA] + ProporcaoA*(VetCDf[IndexA+1] - VetCDf[IndexA]);
			double CLfxz = VetCLf[IndexA] + ProporcaoA*(VetCLf[IndexA+1] - VetCLf[IndexA]);
			double CDfxy = VetCDf[IndexS] + ProporcaoS*(VetCDf[IndexS+1] - VetCDf[IndexS]);
			double CLfxy = VetCLf[IndexS] + ProporcaoS*(VetCLf[IndexS+1] - VetCLf[IndexS]);
			double CDh   = VetCDh[IndexA] + ProporcaoA*(VetCDh[IndexA+1] - VetCDh[IndexA]);
			double CLh   = VetCLh[IndexA] + ProporcaoA*(VetCLh[IndexA+1] - VetCLh[IndexA]);
			double CDv   = VetCDv[IndexS] + ProporcaoS*(VetCDv[IndexS+1] - VetCDv[IndexS]);
			double CLv   = VetCLv[IndexS] + ProporcaoS*(VetCLv[IndexS+1] - VetCLv[IndexS]);

			//Aerodynamic forces applied by the fuselage
			Eigen::MatrixXd Ffxz(3,1), Ffxy(3,1), FIf(3,1);
			
			double kfxy = 0.5 * Ho * sf * pow(Vxy,2);
			double kfxz = 0.5 * Ho * sf * pow(Vxz,2);
			
			Ffxz << -kfxz * CDfxz,            0, kfxz * CLfxz;
			Ffxy << -kfxy * CDfxy, kfxy * CLfxy,            0;
			//std::cout << "Ffxy: " << Ffxy.transpose() << std::endl;
			//std::cout << "kfxy: " << kfxy << std::endl;
			
			
			
			FIf = R_IB * (RBAlpha * Ffxz + RBBeta * Ffxy); // Expressa forças no inercial
			link->AddForceAtRelativePosition( math::Vector3( FIf(0),  FIf(1),  FIf(2)), math::Vector3(  DBf(0),  DBf(1),  DBf(2))); // new
			
			
			//Aerodynamic forces applied by the vertical stabilizer
			double kvxy = 0.5 * Ho * sv * pow(Vxy,2);	
			Eigen::MatrixXd FBv(3,1), FIv(3,1);
			FBv << -kvxy * CDv, kvxy * (CLv + c_r(RudderDeflection)), 0;
			FIv = R_IB*RBBeta*FBv; // Expressa forças no inercial
			link->AddForceAtRelativePosition( math::Vector3( FIv(0),  FIv(1),  FIv(2)), math::Vector3(  DBv(0),  DBv(1),  DBv(2))); // new
			//std::cout << "Vertical Stabilizer" << FIv.transpose() << std::endl;
			
			//Aerodynamic forces applied by the horizontal stabilizer
			double khxz = 0.5 * Ho * sh * pow(Vxz,2);
			Eigen::MatrixXd FBh(3,1), FIh(3,1);
			FBh << -khxz * CDh, 0, khxz * (CLh + c_e(ElevatorDeflection));
			FIh = R_IB*RBAlpha*FBh; // Expressa forças no inercial
			link->AddForceAtRelativePosition( math::Vector3( FIh(0),  FIh(1),  FIh(2)), math::Vector3(  DBh(0),  DBh(1),  DBh(2))); // new	
			//std::cout << "Horizontal Stabilizer" << FIh.transpose() << std::endl;
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
			alpha = atan2((wb-wa),(ub-ua));
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
			beta = atan2((vb-va),(ub-ua));
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
