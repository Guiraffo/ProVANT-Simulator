/*
* File: Aerodinamica.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 09/04/2019
* Description:  This library is responsable to implement the aerodynamic forces of Fuselage, Wings and Tail surfaces on the UAV 4.0. *					The work was based on Daniel Neri phd thesis.
*/				




#include <Aerodinamica.h>

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica::Aerodinamica()//:SaveDataWind(6,1)
	{
		rho = 1.21;	//densidade do ar
		 sf = 0.0146;//area da superficie da fuselagem
		 swr = 0.0023;
		 sh = 0.01437;//area da superficie do estabilizador horizontal
		 sv = 0.013635;//area da superficie do estabilizador vertical
	 	 sTailR = 0.013635;  //area da superficie do Rudder direito
	 	 sTailL = 0.013635; //area da superficie do Rudder esquerdo
		
		
		R_IB.setZero(3,3);
		Rxu1.setZero(3,3);
		Rxu2.setZero(3,3);
		
		wind_B.setZero(3,1); //vento no corpo
		wind_I.setZero(3,1); //vento inercial
        T = 0.0001;
    	u_angle = 0.52;    // V-tail angle wrt de x-aixis of the body frame
    	
		Rxu1 << 1 , 0 , 0,
				0 , cos(-u_angle) , -sin(-u_angle),
				0 , sin(-u_angle) , cos(-u_angle);

		Rxu2 << 1 , 0 , 0,
				0 , cos(u_angle) , -sin(u_angle),
				0 , sin(u_angle) , cos(u_angle);	
				
				
		std::string relativeFile("Wind.txt");
		std::string file = std::getenv("TILT_MATLAB") + relativeFile;
		Wind.startFile(file,"wind");					
				
	}
	Aerodinamica::~Aerodinamica()
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
	std::cout<< "Forcas Aerodinamicas Inicializadas" <<std::endl;
		try
		{
	  		if (!ros::isInitialized())
	  		{
	    			std::cout << "Aerodinamica nao inicializado!" << std::endl;
	    		        return;
	  		}
			
			topic_AileronR = XMLRead::ReadXMLString("topic_AileronR",_sdf);
			topic_AileronL = XMLRead::ReadXMLString("topic_AileronL",_sdf);
			topic_RudderR = XMLRead::ReadXMLString("topic_RudderR",_sdf);
			topic_RudderL = XMLRead::ReadXMLString("topic_RudderL",_sdf);	
			topic_Fr = XMLRead::ReadXMLString("topic_Fr",_sdf);
			topic_Fl = XMLRead::ReadXMLString("topic_Fl",_sdf);		
			
			NameOfLinkMainBody_ = XMLRead::ReadXMLString("MainBody",_sdf);
			NameOfLinkFr_ = XMLRead::ReadXMLString("LinkFr",_sdf);
			NameOfLinkFl_ = XMLRead::ReadXMLString("LinkFl",_sdf);
			NameOfLinkAileronR_ = XMLRead::ReadXMLString("LinkAileronR",_sdf);
			NameOfLinkAileronL_ = XMLRead::ReadXMLString("LinkAileronL",_sdf);	
			NameOfLinkRudderR_ = XMLRead::ReadXMLString("LinkRudderR",_sdf);
			NameOfLinkRudderL_ = XMLRead::ReadXMLString("LinkRudderL",_sdf);									 
			NameOfLinkCentroAerodWr_ = XMLRead::ReadXMLString("CentroAerod_Wr",_sdf);	
			NameOfLinkCentroAerodWl_ = XMLRead::ReadXMLString("CentroAerod_Wl",_sdf);
			NameOfLinkCentroAerodRudR_ = XMLRead::ReadXMLString("CentroAerod_RudR",_sdf);
			NameOfLinkCentroAerodRudL_ = XMLRead::ReadXMLString("CentroAerod_RudL",_sdf);			
			

		
			world = _model->GetWorld();	
			linkFr = _model->GetLink(NameOfLinkFr_);
			linkFl = _model->GetLink(NameOfLinkFl_);				
			linkWr = _model->GetLink(NameOfLinkCentroAerodWr_);
			linkWl = _model->GetLink(NameOfLinkCentroAerodWl_);		
			link = _model->GetLink(NameOfLinkMainBody_);
			linkAileronR = _model->GetLink(NameOfLinkAileronR_);		
			linkAileronL = _model->GetLink(NameOfLinkAileronL_);
			linkRudderR = _model->GetLink(NameOfLinkRudderR_);
			linkRudderL = _model->GetLink(NameOfLinkRudderL_);
			linkRudR = _model->GetLink(NameOfLinkCentroAerodRudR_);
			linkRudL = _model->GetLink(NameOfLinkCentroAerodRudL_);	
			 
			
			
			// update timer
		  	Reset();
		  	updateTimer.Load(world, _sdf);
			updateConnection = updateTimer.Connect(boost::bind(&Aerodinamica::Update, this));

			// subscribers			
			motor_subscriberFR_ = node_handle_.subscribe(topic_Fr, 1, &gazebo::Aerodinamica::CallbackFR, this);
			motor_subscriberFL_ = node_handle_.subscribe(topic_Fl, 1, &gazebo::Aerodinamica::CallbackFL, this);
			motor_subscriberFAileronR_ = node_handle_.subscribe(topic_AileronR, 1, &gazebo::Aerodinamica::CallbackFAileronR, this);
			motor_subscriberFAileronL_ = node_handle_.subscribe(topic_AileronL, 1, &gazebo::Aerodinamica::CallbackFAileronL, this);
			motor_subscriberRudderR_ = node_handle_.subscribe(topic_RudderR, 1, &gazebo::Aerodinamica::CallbackRudderR, this);
			motor_subscriberRudderL_ = node_handle_.subscribe(topic_RudderL, 1, &gazebo::Aerodinamica::CallbackRudderL, this);
		
		

			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Aerodinamica::Reset()
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

	void Aerodinamica::CallbackFR(std_msgs::Float64 msg)
	{
		try
		{
			ignition::math::Vector3d forceR(0,0,msg.data);
			linkFr->AddRelativeForce(forceR);
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
			ignition::math::Vector3d forceL(0,0,msg.data);
			linkFl->AddRelativeForce(forceL);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	
	void Aerodinamica::CallbackFAileronR(std_msgs::Float64 DaR)
	{
	
		try
		{
		
			ElevatorDeflectionR = DaR.data;
				std::cout << "ElevDef: " << ElevatorDeflectionR << std::endl;
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	
	void Aerodinamica::CallbackFAileronL(std_msgs::Float64 DaL)
	{
		try
		{
			ElevatorDeflectionL = DaL.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	void Aerodinamica::CallbackRudderR(std_msgs::Float64 DrR)
	{
		try
		{
			RudderDeflectionR = DrR.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	void Aerodinamica::CallbackRudderL(std_msgs::Float64 DrL)
	{
		try
		{
			RudderDeflectionL = DrL.data;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}			
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	void Aerodinamica::Update()
	{

		try
		{	
			
			
			Ep = link->WorldLinearVel();
			Np = link->WorldAngularVel();
			
			
			//Fuselagem
			common::Time sim_time = world->SimTime();
			boost::mutex::scoped_lock scoped_lock(lock);
			
			// Environment wind
				wind_I << 0, 0, 0;
			
			//cálculo do vento ambiente está errado conferir depois!!!!*****
				wind_B = R_IB.transpose() * wind_I;
				ua = wind_B(0,0);
				va = wind_B(1,0);
				wa = wind_B(2,0);
				
			wind_TailR = Rxu1.transpose() * wind_B;
				ua_TailR = wind_TailR(0,0);
				va_TailR = wind_TailR(1,0);
				wa_TailR = wind_TailR(2,0);
				
				wind_TailL = Rxu2.transpose() * wind_B;
				ua_TailL = wind_TailL(0,0);
				va_TailL = wind_TailL(1,0);
				wa_TailL = wind_TailL(2,0);			
				
				
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
						

			//Velocidade no frame Inercial
			std::vector<double> Vel_I_Fuselagem = GetVel_IS(-0.005,0,0.0326);
			std::vector<double> Vel_I_WingR = GetVel_IS(-0.06,-0.20,0.01);
			std::vector<double> Vel_I_WingL = GetVel_IS(-0.06,0.20,0.01);		
			std::vector<double> Vel_I_TailR = GetVel_IS(-0.375547,-0.065,0.0384);
			std::vector<double> Vel_I_TailL = GetVel_IS(-0.375547,0.065,0.0384);
			
			
			//Velocidades com relacao ao ref. inercial expressadas no ref. do corpo
			std::vector<double> UVW_Fuselagem = GetRelVel(Vel_I_Fuselagem.at(0),Vel_I_Fuselagem.at(1),Vel_I_Fuselagem.at(2),1);
			std::vector<double> UVW_WingR = GetRelVel(Vel_I_WingR.at(0),Vel_I_WingR.at(1),Vel_I_WingR.at(2),2);
			std::vector<double> UVW_WingL = GetRelVel(Vel_I_WingL.at(0),Vel_I_WingL.at(1),Vel_I_WingL.at(2),3);
			std::vector<double> UVW_TailR = GetRelVel(Vel_I_TailR.at(0),Vel_I_TailR.at(1),Vel_I_TailR.at(2),4);
			std::vector<double> UVW_TailL = GetRelVel(Vel_I_TailL.at(0),Vel_I_TailL.at(1),Vel_I_TailL.at(2),5);
			
			
				  
			//Define o angulo alfa para Fuselagem, Asa esquerda e Asa Direita
			alpha_Vec = getAlpha(wa,ua,UVW_Fuselagem,UVW_WingR,UVW_WingL);
			alpha_F = alpha_Vec.at(0);
			alpha_Wr= alpha_Vec.at(1);
			alpha_Wl= alpha_Vec.at(2);
			
			
			
			//Define o angulo beta para Fuselagem, Asa esquerda e Asa Direita 
			beta_Vec = getBeta(va,ua,UVW_Fuselagem);
			beta_F = beta_Vec.at(0);
			
			
			//Define o angulo gama para a Cauda
			gama_Tail = getGama(wa_TailR, ua_TailR, wa_TailL, wa_TailL, UVW_TailR, UVW_TailL); 
			gama_TailR = gama_Tail.at(0);
			gama_TailL = gama_Tail.at(1);
		
			
		/*	static int contador = 0;
			if(contador%20 == 0)
			{
				std::cout << "alpha_F: " << alpha_F<<std::endl;
				std::cout << "alpha_Wr: " << alpha_Wr<<std::endl;
				std::cout << "alpha_Wl: " << alpha_Wl<<std::endl;
				std::cout << "beta_F: "<<beta_F<<std::endl;
			}
			contador++;     */
			
		
			//Define magnitude do vento no plano xz para Fuselagem, Asa esquerda e Asa Direita	
			air_xz_Vec = getAirxz(va,ua,UVW_Fuselagem,UVW_WingR,UVW_WingL);
			double air_xz_F = air_xz_Vec.at(0);
			double air_xz_Wr = air_xz_Vec.at(1);
			double air_xz_Wl = air_xz_Vec.at(2);	
			
			
			//Define magnitude do vento no plano xy para Fuselagem, Asa esquerda e Asa Direita	
			air_xy_Vec = getAirxy(wa,ua,UVW_Fuselagem,UVW_WingR,UVW_WingL);
			double air_xy_F = air_xy_Vec.at(0);
			double air_xy_Wr = air_xy_Vec.at(1);
			double air_xy_Wl = air_xy_Vec.at(2);

			
			//Define magnitude do vento na cauda
			air_Tail_Vec = getAirTail(ua_TailR, wa_TailR, ua_TailL, wa_TailL, UVW_TailR, UVW_TailL);
			double v_TailR = air_Tail_Vec.at(0);
			double v_TailL = air_Tail_Vec.at(1);		
			
			
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			//calculo dos coef. da fuselagem
			double Val = ((alpha_F+3.1416)/0.1)+1.0;
			int IndexA = floor(Val);
			double ProporcaoA = Val - IndexA;

			Val = ((beta_F+3.1416)/0.1)+1.0;
			int IndexS = floor(Val);
			double ProporcaoS = Val - IndexS;

			double CDfa = VetCDf[IndexA] + ProporcaoA*(VetCDf[IndexA+1] - VetCDf[IndexA]);
			double CLfa = VetCLf[IndexA] + ProporcaoA*(VetCLf[IndexA+1] - VetCLf[IndexA]);
			double CDfs = VetCDf[IndexS] + ProporcaoS*(VetCDf[IndexS+1] - VetCDf[IndexS]);
			double CLfs = VetCLf[IndexS] + ProporcaoS*(VetCLf[IndexS+1] - VetCLf[IndexS]);
			
			//Forcas na Fuselagem
			double Fd_xz_F=-0.5*rho*(air_xz_F*air_xz_F)*sf*alpha_F*CDfa;	
			double Fl_xz_F=0.5*rho*(air_xz_F*air_xz_F)*sf*alpha_F*CLfa;
			double Fd_xy_F=-0.5*rho*(air_xy_F*air_xy_F)*sf*beta_F*CDfs;
			double Fl_xy_F=0.5*rho*(air_xy_F*air_xy_F)*sf*beta_F*CLfs;
			Eigen::MatrixXd Ryalpha(3,3);
			Ryalpha<<	cos(alpha_F), 0 , sin(alpha_F),
						0, 1, 0 ,
						-sin(alpha_F),0,cos(alpha_F);
			Eigen::MatrixXd Rybeta(3,3);			
			Rybeta<<	cos(beta_F) , 0 , 0,
						0    ,1   ,    0,
						-sin(beta_F) , 0 , cos(beta_F);
			Eigen::VectorXd comp1_forca(3);
			comp1_forca << Fd_xz_F,0,Fl_xz_F;
			Eigen::VectorXd comp2_forca(3);
			comp2_forca << Fd_xy_F,Fl_xy_F,0;
			
			Eigen::VectorXd Forca_F(3);
			Forca_F = Ryalpha*comp1_forca + Rybeta*comp2_forca;
			Ff.x=Forca_F(0);
			Ff.y=Forca_F(1);
			Ff.z=Forca_F(2);
			link->AddForce(Ff);	
			
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			
			//calculo dos coef. da asa
			double Val_Wr = ((alpha_Wr+3.1416)/0.1)+1.0;
			int IndexAWr = floor(Val_Wr);
			double ProporcaoAWr = Val_Wr - IndexAWr;
			
			double Val_Wl = ((alpha_Wl+3.1416)/0.1)+1.0;
			int IndexAWl = floor(Val_Wl);
			double ProporcaoAWl = Val_Wl - IndexAWl;
			
			double CDWr  = VetCDh[IndexAWr] + ProporcaoAWr*(VetCDh[IndexAWr+1] - VetCDh[IndexAWr]);
			double CLWr  = VetCLh[IndexAWr] + ProporcaoAWr*(VetCLh[IndexAWr+1] - VetCLh[IndexAWr]);
			double CDWl  = VetCDh[IndexAWl] + ProporcaoAWl*(VetCDh[IndexAWl+1] - VetCDh[IndexAWl]);
			double CLWl  = VetCLh[IndexAWl] + ProporcaoAWl*(VetCLh[IndexAWl+1] - VetCLh[IndexAWl]);			


			
			//Forcas nas Asas	
		    double Fd_Wr = -0.5 * rho * (air_xz_Wr * air_xz_Wr) *	swr *  CDWr * alpha_Wr;
		    double Fl_Wr =  0.5 * rho * (air_xz_Wr * air_xz_Wr) * swr * (CLWr * alpha_Wr + c_AileronR(ElevatorDeflectionR));
		    double Fd_Wl = -0.5 * rho * (air_xz_Wl * air_xz_Wl) *	swr *  CDWl * alpha_Wl;
		    double Fl_Wl =  0.5 * rho * (air_xz_Wr * air_xz_Wr) * swr * (CLWl * alpha_Wl + c_AileronL(ElevatorDeflectionR));
		    
			Eigen::MatrixXd RyalphaWr(3,3);
			RyalphaWr<<	   cos(alpha_Wr),0,sin(alpha_Wr),
							0  ,  1  ,  0,
						-sin(alpha_Wr),0,cos(alpha_Wr);
			Eigen::MatrixXd RyalphaWl(3,3);
			RyalphaWl<<	  cos(alpha_Wl),0,sin(alpha_Wl),
							0  ,  1  ,  0,
						-sin(alpha_Wl),0,cos(alpha_Wl);			
			Eigen::VectorXd comp1_forcaWr(3);
			comp1_forcaWr << Fd_Wr,0,Fl_Wl;
			Eigen::VectorXd comp1_forcaWl(3);
			comp1_forcaWl << Fd_Wl,0,Fl_Wl;
			
			Eigen::VectorXd Forca_Wr(3);
			Forca_Wr = 	RyalphaWr*comp1_forcaWr;
			FWr.x = Forca_Wr(0);		
			FWr.y = Forca_Wr(1);
			FWr.z = Forca_Wr(2);
		   //	std::cout<<"FORCA WR: "<<FWr<<std::endl;
			linkWr->AddForce(FWr);
			
			Eigen::VectorXd Forca_Wl(3);
			Forca_Wl = 	RyalphaWl*comp1_forcaWl;   
			FWl.x = Forca_Wl(0);		
			FWl.y = Forca_Wl(1);
			FWl.z = Forca_Wl(2);
			//	 std::cout<<"FORCA WL: "<<FWl<<std::endl;
			linkWl->AddForce(FWl) ; 
				   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			   
			
			//Calculo dos coeficientes da Cauda	   
			double Val_RudR = ((gama_TailR + 3.1416)/0.1)+1.0;   
			int IndexRudR = floor(Val_RudR);
			double ProporcaoRudR = Val_RudR - IndexRudR;
			
			double Val_RudL = ((gama_TailL + 3.1416)/0.1)+1.0;
			int IndexRudL = floor(Val_RudL);
			double ProporcaoRudL = Val_RudL - IndexRudL;	   
				   
			double CDRudR  = VetCDv[IndexRudR] + ProporcaoRudR*(VetCDv[IndexRudR+1] - VetCDv[IndexRudR]);
			double CLRudR = VetCLv[IndexRudR] + ProporcaoRudR*(VetCLv[IndexRudR+1] - VetCLv[IndexRudR]);	
			double CDRudL  = VetCDv[IndexRudL] + ProporcaoRudL*(VetCDv[IndexRudL+1] - VetCDv[IndexRudL]);
			double CLRudL = VetCLv[IndexRudL] + ProporcaoRudL*(VetCLv[IndexRudL+1] - VetCLv[IndexRudL]);	   
			
			double Fd_TailR = -0.5 * rho * (v_TailR * v_TailR) * sTailR * CDRudR * gama_TailR;
			double Fl_TailR = 0.5 * rho * (v_TailR * v_TailR) * sTailR * (CLRudR * gama_TailR + c_RudR(RudderDeflectionR));
			double Fd_TailL = -0.5 * rho * (v_TailL * v_TailL) * sTailL * CDRudL * gama_TailL;
			double Fl_TailL = 0.5 * rho * (v_TailL * v_TailL) * sTailL * (CLRudL * gama_TailL + c_RudL(RudderDeflectionL));
			
			//Forca na cauda Direita
			Eigen::MatrixXd RygamaR(3,3);
			RygamaR << cos(gama_TailR) , 0 , sin(gama_TailR),
					   0, 1 , 0,
					   -sin(gama_TailR),0, cos(gama_TailR);
			Eigen::VectorXd comp1_forcaTailR(3);
			comp1_forcaTailR << Fd_TailR, Fl_TailR, 0;
			Eigen::VectorXd F_TailR(3);
			F_TailR = Rxu1 * RygamaR * comp1_forcaTailR;
			FTailR.x = F_TailR(0);
			FTailR.y = F_TailR(1);
			FTailR.z = F_TailR(2);			
			
			linkRudR->AddForce(FTailR);
			
			//Forca na cauda Esquerda
			
			Eigen::MatrixXd RygamaL(3,3);
			RygamaR << cos(gama_TailL) , 0 , sin(gama_TailL),
					   0, 1 , 0,
					   -sin(gama_TailL),0, cos(gama_TailL);
			Eigen::VectorXd comp1_forcaTailL(3);
			comp1_forcaTailR << Fd_TailL, Fl_TailL, 0;
			Eigen::VectorXd F_TailL(3);
			F_TailL = Rxu2 * RygamaL * comp1_forcaTailL;
			FTailL.x = F_TailL(0);
			FTailL.y = F_TailL(1);
			FTailL.z = F_TailL(2);			
			
			linkRudL->AddForce(FTailL);				  		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	
	} 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
		//velocidade da superficie aerodinamica no sis. coord. Inercial
		
	  std::vector<double> Aerodinamica::GetVel_IS(double a, double b, double c){
	
		Eigen::VectorXd dist_BS(3);
		dist_BS << a, b, c;

		ignition::math::Pose3d pose = link->WorldPose();
		phi = pose.rot.GetAsEuler().x;
		theta = pose.rot.GetAsEuler().y;
		psi = pose.rot.GetAsEuler().z;
		
		
		//Define Wn
		Eigen::MatrixXd Wn(3,3);
		Wn <<   1,0,-sin(theta),
				0,cos(phi),cos(theta)*sin(phi),
				0,-sin(phi),cos(phi)*cos(theta);
		
		//Define Transformação do Body frame para o Inercial frame
		R_IB << (cos(psi)*cos(theta)), (cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi)), (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)),
				(cos(theta)*sin(psi)), (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)),
				(-sin(theta)), (cos(theta)*sin(phi)), (cos(phi)*cos(theta));
				
		//Define Skew Matrix	
			Eigen::MatrixXd SkewMatrix(3,3);
			SkewMatrix << 0, -dist_BS(2), dist_BS(1),
						  dist_BS(2),0,-dist_BS(0),
						  -dist_BS(1),dist_BS(0),0;
						  
		
			
		
		//Velocidades no Inertial frame
		Eigen::VectorXd Ndot(3);
		Ndot<<Np.x,Np.y,Np.z;		
		Eigen::VectorXd Edot(3);
		Edot<<Ep.x,Ep.y,Ep.z;
		
		Eigen::VectorXd Vel_IS(3); 
		 Vel_IS = -R_IB*SkewMatrix*Wn*Ndot + Edot;
		 		
		std::vector<double> A;
		A.push_back(Vel_IS(0));
		A.push_back(Vel_IS(1));
		A.push_back(Vel_IS(2));		
		
		return 	A;  				
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	    std::vector<double> Aerodinamica::GetRelVel(double d, double e, double f,int g)
{
	Eigen::VectorXd VelIN(3);			//Velocity of the aerodynamic center wrt the inertial frame. N being the frames of Wr,Wl,TailR,
	VelIN<< d,e,f;					   // TailL and Fuselage				 
	
	
	switch(g){
	case 1: {
	Eigen::VectorXd UVW_F(3);
	UVW_F = R_IB.transpose()*VelIN;
	std::vector<double> uvw_F;
	uvw_F.push_back(UVW_F(0));
	uvw_F.push_back(UVW_F(1));
	uvw_F.push_back(UVW_F(2));
	return uvw_F;
			}
	break;
	case 2: {
	double e = 0.087;
	Eigen::MatrixXd Rye(3,3);
	Rye <<  cos(e),0,sin(e),
			0,1,0,
			-sin(e),0,cos(e);
	Eigen::VectorXd UVW_Wr(3);
	UVW_Wr = (R_IB*Rye).transpose()*VelIN;
	std::vector<double>	uvw_Wr;
	uvw_Wr.push_back(UVW_Wr(0));
	uvw_Wr.push_back(UVW_Wr(1));
	uvw_Wr.push_back(UVW_Wr(2));
	return uvw_Wr;
			}
	break;
	case 3:{
	double e2 = 0.087;
	Eigen::MatrixXd Rye2(3,3);
	Rye2 << cos(e)  ,0,  sin(e),
			  0    , 1  ,0,
			-sin(e) , 0 , cos(e);
	Eigen::VectorXd UVW_Wl(3);
	UVW_Wl = (R_IB*Rye2).transpose() * VelIN;
	std::vector<double>	uvw_Wl;
	uvw_Wl.push_back(UVW_Wl(0));
	uvw_Wl.push_back(UVW_Wl(1));
	uvw_Wl.push_back(UVW_Wl(2));
	return uvw_Wl;
			}
	break;
	case 4:{

	Eigen::MatrixXd R_ITailR(3,3);
	R_ITailR = R_IB * Rxu1;
	Eigen::VectorXd UVW_TailR(3);
	UVW_TailR = R_ITailR.transpose() * VelIN;	
	
	std::vector<double> uvw_TailR;	
	uvw_TailR.push_back(UVW_TailR(0));
	uvw_TailR.push_back(UVW_TailR(1));
	uvw_TailR.push_back(UVW_TailR(2));
	return uvw_TailR;
		   }
	break;
	case 5:{

	Eigen::MatrixXd R_ITailL(3,3);
	R_ITailL = R_IB * Rxu2;
	Eigen::VectorXd UVW_TailL(3);
	UVW_TailL = R_ITailL.transpose() * VelIN;	
	
	std::vector<double> uvw_TailL;	
	uvw_TailL.push_back(UVW_TailL(0));
	uvw_TailL.push_back(UVW_TailL(1));
	uvw_TailL.push_back(UVW_TailL(2));
	return uvw_TailL;
		   }
	break;	  	   
	}		
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
		std::vector<double> Aerodinamica::getAlpha( double wa, double ua, std::vector<double> ha, std::vector<double> ia,std::vector<double> ja)
{
	try
	{
		double alpha_F,alpha_Wr,alpha_Wl;
		
			alpha_F = -atan2((ha.at(2)-wa),(ha.at(0)-ua));
			if(alpha_F > 3.0584)
			{
				alpha_F = 3.0584;
			}else if( alpha_F < -3.1416){
				alpha_F = -3.1416;
			}
			alpha_Wr = -atan2((ia.at(2)-wa),(ia.at(0)-ua));
			if(alpha_Wr > 3.0584)
			{
				alpha_Wr = 3.0584;
			}else if( alpha_Wr < -3.1416){
				alpha_Wr = -3.1416;
			}
			alpha_Wl = -atan2((ja.at(2)-wa),(ja.at(0)-ua));
			if(alpha_Wl > 3.0584)
			{
				alpha_Wl = 3.0584;
			}else if( alpha_Wl < -3.1416){
				alpha_Wl = -3.1416;
			}
			
			
			std::vector<double> AlphaVector;
			AlphaVector.push_back(alpha_F);
			AlphaVector.push_back(alpha_Wr);
			AlphaVector.push_back(alpha_Wl);
				return AlphaVector;
			
			
		
		}	
			catch(std::exception& e)	
		{
			std::cout << e.what() << std::endl;
		}
	}
		
	
	std::vector<double> Aerodinamica::getBeta(double va, double ua, std::vector<double> hb)
	{
		try
		{
			double beta_F,beta_Wr,beta_Wl;
			
			beta_F = -atan2((hb.at(1)-va),(hb.at(0)-ua));		
			if(beta_F > 3.0584)
			{
				beta_F = 3.0584;
			}else if( beta_F < -3.1416){
				beta_F = -3.1416;
			}	
			
			std::vector<double> BetaVector;
			BetaVector.push_back(beta_F);	
			return BetaVector;	
			
		}
		catch(std::exception& e)	
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	std::vector<double> Aerodinamica::getGama(double , double , double, double, std::vector<double> q, std::vector<double> r){
	try
	{
	double gama_R, gama_L;
	gama_R = -atan2((q.at(2)-wa_TailR),(q.at(0)-ua_TailR));
			if(gama_R > 3.0584)
			{
				gama_R = 3.0584;
			}else if( gama_R < -3.1416){
				gama_R = -3.1416;
			}
	gama_L = -atan2((r.at(2)-wa_TailL),(r.at(0)-ua_TailL));
		    if(gama_L > 3.0584)
			{
				gama_L = 3.0584;
			}else if( gama_L < -3.1416){
				gama_L = -3.1416;
			}
			
	std::vector<double> GamaVector;
	GamaVector.push_back(gama_R);
	GamaVector.push_back(gama_L);
	return GamaVector;
	}
	catch(std::exception& e)	
		{
			std::cout << e.what() << std::endl;
		}
	}	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<double> Aerodinamica::getAirxz( double wa, double ua, std::vector<double> k,std::vector<double> l,std::vector<double> m)
	{
		try
		{
		double air1,air2,air3;
			air1 = sqrt(pow(k.at(2)-wa,2)+ pow(k.at(0)-ua,2));
			air2 = sqrt(pow(l.at(2)-wa,2) + pow(l.at(0)-ua,2));
			air3 = sqrt(pow(m.at(2)-wa,2) + pow(m.at(0)-ua,2));
			
		std::vector<double> Air_Vector;
		Air_Vector.push_back(air1);
		Air_Vector.push_back(air2);
		Air_Vector.push_back(air3);
		return	Air_Vector;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}

	std::vector<double> Aerodinamica::getAirxy( double va, double ua,std::vector<double> n,std::vector<double> o,std::vector<double> p )
	{
		try
		{
		double air11,air22,air33;
			air11 = sqrt(pow(n.at(1)-va,2) + pow(n.at(0)-ua,2));
			air22 = sqrt(pow(o.at(1)-va,2) + pow(o.at(0)-ua,2));
			air33 = sqrt(pow(p.at(1)-va,2) + pow(p.at(0)-ua,2));
			
		std::vector<double> Air_Vector2;
		Air_Vector2.push_back(air11);
		Air_Vector2.push_back(air22);
		Air_Vector2.push_back(air33);
		return Air_Vector2;	
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}
	std::vector<double> Aerodinamica::getAirTail(double, double, double, double, std::vector<double> qq, std::vector<double> rr){
	try
	{
	double airTailR, airTailL;
	airTailR = sqrt(pow(qq.at(0)-ua_TailR,2) + pow(qq.at(2)-wa_TailR,2));
	airTailL = sqrt(pow(rr.at(0)-ua_TailL,2) + pow(rr.at(2)-wa_TailL,2));
	
	std::vector<double> Air_Tail_Vector;
	Air_Tail_Vector.push_back(airTailR);
	Air_Tail_Vector.push_back(airTailL);
	return Air_Tail_Vector;
	}
	catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}				
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	

	double Aerodinamica::c_AileronR(double DeR)
	{
		try
		{	
			return 2.1873375*DeR;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	double Aerodinamica::c_AileronL(double DeL)
	{
		try
		{
			return 2.1873375*DeL;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	
	
	
	double Aerodinamica::c_RudR(double DrR)
	{
		try
		{
			return 1.165*DrR;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}
	double Aerodinamica::c_RudL(double DrL)
	{
		try
		{
			return 1.165*DrL;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}


	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	
	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}
				
