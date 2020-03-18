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
#include "/usr/include/eigen3/Eigen/Eigen"
//#include "SkewSymmetricMatrix.h"

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica::Aerodinamica(): DBf(3), DBwr(3), DBwl(3), DBtr(3), DBtl(3), RI_B(3,3), Wn(3,3), WI_IB(3), PhipThetapPsip(3), XpYpZp(3), EnvironmentWind(3), RBmiL(3,3), RBmiR(3,3), RBwdL(3,3), RBwdR(3,3), dPI_f(3), dPI_wr(3), dPI_wl(3), dPI_tr(3), dPI_tl(3), UVWf(3), UVWfa(3), UVWwr(3), UVWwra(3), UVWwl(3), UVWwla(3), UVWtr(3), UVWtra(3), UVWtl(3), UVWtla(3), RAlphaf(3,3), RBetaf(3,3), 
	Fxz(3), Fxy(3), Forca_F(3), FWrxz(3), FWlxz(3), Forca_Wr(3), Forca_Wl(3), RAlphawr(3,3), RAlphawl(3,3), RGammatr(3,3), RGammatl(3,3), ForcaTailR(3), ForcaTailL(3), F_TailR(3), F_TailL(3)
	{
		rho = 1.21;	//densidade do ar
		sf = 0.0946;    //area da superficie da fuselagem
		sw = 0.1123;   //area da superficie da asa
	 	st = 0.0492;  //area da superficie do Rudder esquerdo
        	T = 0.0001;	
        	mi = 0.5236; // tail deflection rad
        	wd = 0.0873; // wing diedral rad
				
		//std::string relativeFile("Wind.txt");
		//std::string file = std::getenv("TILT_MATLAB") + relativeFile;
		//Wind.startFile(file,"wind");					
				
	}
	Aerodinamica::~Aerodinamica()
	{	//Wind.endFile();
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
			NameOfLinkCentroAerodF_ = XMLRead::ReadXMLString("CentroAerod_F",_sdf);
			NameOfLinkCentroAerodWr_ = XMLRead::ReadXMLString("CentroAerod_Wr",_sdf);	
			NameOfLinkCentroAerodWl_ = XMLRead::ReadXMLString("CentroAerod_Wl",_sdf);
			NameOfLinkCentroAerodRudR_ = XMLRead::ReadXMLString("CentroAerod_RudR",_sdf);
			NameOfLinkCentroAerodRudL_ = XMLRead::ReadXMLString("CentroAerod_RudL",_sdf);			
			

		
			world = _model->GetWorld();	
			MainBody = _model->GetLink(NameOfLinkMainBody_); 
			linkFr = _model->GetLink(NameOfLinkFr_);
			linkFl = _model->GetLink(NameOfLinkFl_);
			linkF = _model->GetLink(NameOfLinkCentroAerodF_);				
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
			math::Vector3 forceR(0,0,msg.data);
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
			math::Vector3 forceL(0,0,msg.data);
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
				//std::cout << "ElevDef: " << ElevatorDeflectionR << std::endl;
			
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
			
			
			//Computing the wind properties
			
			math::Vector3 Linear = link->GetWorldLinearVel();
			math::Vector3 Angular = link->GetWorldAngularVel();
			math::Pose pose = link->GetWorldPose();

			//Final - Obtido com base no email do Sergio
			EnvironmentWind << 0, 0, 0;
			DBf  << 0.0512, 0, 0.1;
			DBwr << 0.0512, -0.31, 0.1;
			DBwl << 0.0512,  0.31, 0.1;
			DBtr << -0.3967, -0.168, 0.148;
			DBtl << -0.3967,  0.168, 0.148;
			
			//computing configuration variables
			
			Phi = pose.rot.GetAsEuler( ).x;
			Theta = pose.rot.GetAsEuler( ).y;
			Psi = pose.rot.GetAsEuler( ).z;
			
			//computing transformation matrices
			RI_B <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + 					cos(Phi)*cos(Psi)*sin(Theta)),(cos(Theta)*sin(Psi)), 
				(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), 						 					(cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)), (cos(Theta)*sin(Phi)), (cos(Phi)*cos(Theta));
			Wn << 1.0,         0.0,          -sin(Theta), 
			       0.0,  cos(Phi),  cos(Theta)*sin(Phi),
	  	               0.0, -sin(Phi),  cos(Phi)*cos(Theta);
	  	               
       			//Tail R and L - Rotation w.r.t x-axis
			RBmiL  <<  1,       0,        0,
				   0, cos(mi), -sin(mi),
				   0, sin(mi),  cos(mi);
			     
			RBmiR <<  1,        0,         0,
				  0, cos(-mi), -sin(-mi),
				  0, sin(-mi),  cos(-mi);
				 
			//Diedral - Wings R and L w.r.t x-axis
			RBwdL <<  1,       0,        0,
				  0, cos(wd), -sin(wd),
				  0, sin(wd),  cos(wd);
			     
			RBwdR <<  1,        0,         0,
				  0, cos(-wd), -sin(-wd),
				  0, sin(-wd),  cos(-wd);
			
			//Computing [phidot thetadot psidot]
			WI_IB << Angular.x, Angular.y, Angular.z;
			PhipThetapPsip = Wn.inverse() * RI_B.transpose() * WI_IB;
			
			//Computing [Xdot Ydot Zdot]
			XpYpZp << Linear.x, Linear.y, Linear.z;
			
			//Compute the velocity of the aerodynamic centers expressed in the Inertial frame
			dPI_f  << -RI_B*SkewSymmetricMatrix( DBf  )*Wn*PhipThetapPsip + XpYpZp;
			dPI_wr << -RI_B*SkewSymmetricMatrix( DBwr )*Wn*PhipThetapPsip + XpYpZp;
			dPI_wl << -RI_B*SkewSymmetricMatrix( DBwl )*Wn*PhipThetapPsip + XpYpZp;
			dPI_tr << -RI_B*SkewSymmetricMatrix( DBtr )*Wn*PhipThetapPsip + XpYpZp;
			dPI_tl << -RI_B*SkewSymmetricMatrix( DBtl )*Wn*PhipThetapPsip + XpYpZp;
			
			//----------Computing Properties of Relative wind for fuselage---------%

			UVWf  = RI_B.transpose() * dPI_f; //Compute UVW aerodynamic center of fuselage
			UVWfa = RI_B.transpose() * EnvironmentWind; //Compute UVW enviroment wind-speed for fuselage

			double Vfxz = pow(pow(UVWf(2)-UVWfa(2),2) + pow(UVWf(0)-UVWfa(0),2) , 0.5); //Magnitude x-z axis
			double Vfxy = pow(pow(UVWf(1)-UVWfa(1),2) + pow(UVWf(0)-UVWfa(0),2) , 0.5); //Magnitude x-y axis

			double Alphaf = atan2(UVWf(2)-UVWfa(2),UVWf(0)-UVWfa(0)); //Orientation - Angle of attack
			double Betaf =  atan2(UVWf(1)-UVWfa(1),UVWf(0)-UVWfa(0)); //Orientation - Side slip angle
			
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << "Vfxz:" << Vfxz << "  Vfxy:" << Vfxy << std::endl;
			std::cout << "Alphaf:" << Alphaf << "  Betaf:" << Betaf << std::endl;
			//----------Computing Properties of Relative wind for wings---------%

			//Wing right
			UVWwr = RBwdR.transpose() * RI_B.transpose() * dPI_wr; //Compute UVW aerodynamic center
			UVWwra = RBwdR.transpose() * RI_B.transpose() * EnvironmentWind; //Compute UVW enviroment wind-speed
			double Vwr = pow( pow(UVWwr(2)-UVWwra(2) , 2) + pow(UVWwr(0)-UVWwra(0) , 2) , 0.5); //Magnitude
			double Alphawr = atan2( UVWwr(2)-UVWwra(2) , UVWwr(0)-UVWwra(0) ); //Orientation

			//Wing left
			UVWwl = RBwdL.transpose() * RI_B.transpose() * dPI_wl; //Compute UVW aerodynamic center
			UVWwla = RBwdL.transpose() * RI_B.transpose() * EnvironmentWind; //Compute UVW enviroment wind-speed
			double Vwl = pow( pow(UVWwl(2)-UVWwla(2),2) + pow(UVWwl(0)-UVWwla(0),2), 0.5); //Magnitude
			double Alphawl = atan2(UVWwl(2)-UVWwla(2), UVWwl(0)-UVWwla(0)); //Orientation
			
			std::cout << std::endl << "Vwr:" << Vwr << "  Vwl:" << Vwl << std::endl;
			std::cout << "Alphawr:" << Alphawr << "  Alphawl:" << Alphawl << std::endl;

//			//----------Computing Properties of Relative wind for V-tail---------%

			//Tail right
			UVWtr = RBmiR.transpose() * RI_B.transpose() * dPI_tr; //Compute UVW aerodynamic center
			UVWtra = RBmiR.transpose() * RI_B.transpose() * EnvironmentWind; //Compute UVW enviroment wind-speed
			double Vtr = pow( pow(UVWtr(2)-UVWtra(2),2) + pow(UVWtr(0)-UVWtra(0),2), 0.5); //Magnitude
			double Gammatr = atan2( UVWtr(2)-UVWtra(2) , UVWtr(0)-UVWtra(0) ); //Orientation

			//Tail left
			UVWtl = RBmiL.transpose() * RI_B.transpose() * dPI_tl; //Compute UVW aerodynamic center
			UVWtla = RBmiL.transpose() * RI_B.transpose() * EnvironmentWind; //Compute UVW enviroment wind-speed
			double Vtl = pow( pow(UVWtl(2)-UVWtla(2),2) + pow(UVWtl(0)-UVWtla(0),2) , 0.5); //Magnitude
			double Gammatl = atan2( UVWtl(2)-UVWtla(2) , UVWtl(0)-UVWtla(0) );  //Orientation
			
			
			std::cout << std::endl << "Vtr:" << Vtr << "  Vtl:" << Vtl << std::endl;
			std::cout << "Gammatr:" << Gammatr << "  Gammatl:" << Gammatl << std::endl;
			
			
			
			
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			
			//calculo dos coef. da fuselagem
			double Val = ((-Alphaf+3.1416)/0.1);
			int Index = floor(Val);
			double Proporcao = Val - Index;
			
			double CDfa = VetCDf[Index] + Proporcao*(VetCDf[Index+1] - VetCDf[Index]);
			double CLfa = VetCLf[Index] + Proporcao*(VetCLf[Index+1] - VetCLf[Index]);
			
			Val = ((-Betaf+3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;
			//Index = Index + 1;

			double CDfs = VetCDf[Index] + Proporcao*(VetCDf[Index+1] - VetCDf[Index]);
			double CLfs = VetCLf[Index] + Proporcao*(VetCLf[Index+1] - VetCLf[Index]);
			
			//std::cout << std::endl << "CLfs: " << CLfs << "Betaf: " << Betaf << "Index: " << Index << "Proporcao: " << Proporcao  << std::endl;
			
			//calculo dos coef. da asa
			Val = ((-Alphawr+3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;
			
			double CDWr  = VetCDW[Index] + Proporcao*(VetCDW[Index+1] - VetCDW[Index]);
			double CLWr  = VetCLW[Index] + Proporcao*(VetCLW[Index+1] - VetCLW[Index]);

			
			Val = ((-Alphawl+3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;
			
			double CDWl  = VetCDW[Index] + Proporcao*(VetCDW[Index+1] - VetCDW[Index]);
			double CLWl  = VetCLW[Index] + Proporcao*(VetCLW[Index+1] - VetCLW[Index]);	
			

			//Calculo dos coeficientes da Cauda	   
			Val = ((-Gammatr + 3.1416)/0.1);   
			Index = floor(Val);
			Proporcao = Val - Index;
			
			double CDTr  = VetCDt[Index] + Proporcao*(VetCDt[Index+1] - VetCDt[Index]);
			double CLTr  = VetCLt[Index] + Proporcao*(VetCLt[Index+1] - VetCLt[Index]);	

			Val = ((-Gammatl + 3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;	   
			
			double CDTl  = VetCDt[Index] + Proporcao*(VetCDt[Index+1] - VetCDt[Index]);
			double CLTl  = VetCLt[Index] + Proporcao*(VetCLt[Index+1] - VetCLt[Index]);
			
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			


			//Forcas atuantes na Fuselagem
			double Fd_xz_F = -0.5 * rho * (Vfxz*Vfxz) * sf * CDfa;	
			double Fl_xz_F =  0.5 * rho * (Vfxz*Vfxz) * sf * CLfa;
			double Fd_xy_F = -0.5 * rho * (Vfxy*Vfxy) * sf * CDfs;
			double Fl_xy_F =  0.5 * rho * (Vfxy*Vfxy) * sf * CLfs;
			//Rotation matrices that map from wind to the aerodynamic center frame
		
			
			RAlphaf << cos(Alphaf),  0, sin(Alphaf),
					     0,  1,          0 ,
				  -sin(Alphaf),  0, cos(Alphaf);
				 
			RBetaf <<  cos(Betaf), -sin(Betaf),   0,
				   sin(Betaf),  cos(Betaf),   0,
				            0,           0,   1;
				
			//compute aerodynamic forces applied by the fuselage
			Fxz << Fd_xz_F, 0, Fl_xz_F;
			Fxy << Fd_xy_F, Fl_xy_F, 0;
			
			//std::cout << std::endl << "Fd_xy_F: " << Fd_xy_F << " Fl_xy_F: " << Fl_xy_F << std::endl;
			
			Forca_F = RAlphaf*Fxz + RBetaf*Fxy;
			
//			// apply the aerodynamic forces
//			Ff.x = Forca_F(0);
//			Ff.y = Forca_F(1);
//			Ff.z = Forca_F(2);
//			
//			math::Vector3 DBf_V3;
//			DBf_V3.x = DBf(0);
//			DBf_V3.y = DBf(1);
//			DBf_V3.z = DBf(2);

			//workswell
			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_F(0), Forca_F(1), Forca_F(2)), math::Vector3( DBf(0), DBf(1), DBf(2)));
			
			//linkF->AddForce(Ff);

			std::cout << std::endl << "Ff.x: " << Forca_F(0) << " Ff.y: " << Forca_F(1) << " Ff.z: " << Forca_F(2) << std::endl;	
			
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			
			//Forcas nas Asas	
		    	double Fd_Wr = -0.5 * rho * (Vwr * Vwr) *   sw *  CDWr;
		    	double Fl_Wr =  0.5 * rho * (Vwr * Vwr) *   sw * (CLWr + c_AileronR(ElevatorDeflectionR));
		    	double Fd_Wl = -0.5 * rho * (Vwl * Vwl) *   sw *  CDWl;
		    	double Fl_Wl =  0.5 * rho * (Vwl * Vwl) *   sw * (CLWl + c_AileronL(ElevatorDeflectionL));
			
			RAlphawr <<	cos(Alphawr),  0  , sin(Alphawr),
					         0  ,  1  ,            0,
				       -sin(Alphawr),  0  , cos(Alphawr);
			
			RAlphawl <<     cos(Alphawl),  0  , sin(Alphawl),
					         0  ,  1  ,            0,
				       -sin(Alphawl),  0  , cos(Alphawl);			
			
			FWrxz << Fd_Wr, 0, Fl_Wr;
			FWlxz << Fd_Wl, 0, Fl_Wl;
						
			Forca_Wr = RBwdR*RAlphawr*FWrxz; 
//			FWr.x = Forca_Wr(0);		
//			FWr.y = Forca_Wr(1);
//			FWr.z = Forca_Wr(2);
//			
			Forca_Wl = RBwdL*RAlphawl*FWlxz;
//			FWl.x = Forca_Wl(0);		
//			FWl.y = Forca_Wl(1);
//			FWl.z = Forca_Wl(2);
			
			//workswell
			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_Wr(0), Forca_Wr(1), Forca_Wr(2)), math::Vector3( DBwr(0), DBwr(1), DBwr(2)));
      			
      			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_Wl(0), Forca_Wl(1), Forca_Wl(2)), math::Vector3( DBwl(0), DBwl(1), DBwl(2)));
							      
							      

//			linkWr->AddForce(FWr);
//			linkWl->AddForce(FWl); 
			std::cout << std::endl << "FWr.x: " << Forca_Wr(0) << " FWr.y: " << Forca_Wr(1) << " FWr.z: " << Forca_Wr(2) << std::endl;
			std::cout << std::endl << "FWl.x: " << Forca_Wl(0) << " FWl.y: " << Forca_Wl(1) << " FWl.z: " << Forca_Wl(2) << std::endl;
//				   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			   

			double Fd_TR = -0.5 * rho * (Vtr * Vtr) * st * CDTr;
			double Fl_TR =  0.5 * rho * (Vtr * Vtr) * st * (CLTr + c_RudR(RudderDeflectionR));
			double Fd_TL = -0.5 * rho * (Vtl * Vtl) * st * CDTl;
			double Fl_TL =  0.5 * rho * (Vtl * Vtl) * st * (CLTl + c_RudL(RudderDeflectionL));
			
			//Forca na cauda Direita
			RGammatr << cos(Gammatr) , 0 , sin(Gammatr),
					       0 , 1 ,            0,
			           -sin(Gammatr) , 0 , cos(Gammatr);
			            
			            
			RGammatl <<  cos(Gammatl), 0 , sin(Gammatl),
					        0, 1 ,            0,
			            -sin(Gammatl), 0 , cos(Gammatl);
			
			ForcaTailR << Fd_TR, 0, Fl_TR;
			
			F_TailR = RBmiR * RGammatr * ForcaTailR;
			//F_TailR = RGammatr * ForcaTailR;
//			FTr.x = F_TailR(0);
//			FTr.y = F_TailR(1);
//			FTr.z = F_TailR(2);			
			
			//Forca na cauda Esquerda
			
			ForcaTailL << Fd_TL, 0, Fl_TL;
			
			F_TailL = RBmiL * RGammatl * ForcaTailL;
			//F_TailL = RGammatl * ForcaTailL;
//			FTl.x = F_TailL(0);
//			FTl.y = F_TailL(1);
//			FTl.z = F_TailL(2);	
//			
//			
//			math::Vector3 DBtl_V3;
//			DBtl_V3.x = DBtl(0);
//			DBtl_V3.y = DBtl(1);
//			DBtl_V3.z = DBtl(2);
//			
//			math::Vector3 DBtr_V3;
//			DBtr_V3.x = DBtr(0);
//			DBtr_V3.y = DBtr(1);
//			DBtr_V3.z = DBtr(2);
			
			
			//workswell
			MainBody->AddForceAtRelativePosition( math::Vector3(F_TailR(0), F_TailR(1), F_TailR(2)), math::Vector3( DBtr(0), DBtr(1), DBtr(2)));
      			MainBody->AddForceAtRelativePosition( math::Vector3(F_TailL(0), F_TailL(1), F_TailL(2)), math::Vector3( DBtl(0), DBtl(1), DBtl(2)));
      			
      			
			
//			//MainBody->AddForceAtRelativePosition(FTl, DBtl_V3);
//			MainBody->AddForceAtRelativePosition(FTl, math::Vector3(DBtl(0),DBtl(1),DBtl(2)));
//			MainBody->AddForceAtRelativePosition(FTr, DBtr_V3);
			
			//linkRudR->AddForce(FTr);
			//linkRudL->AddForce(FTl);
			std::cout << std::endl << "FTr.x: " << F_TailR(0) << " FTr.y: " << F_TailR(1) << " FTr.z: " << F_TailR(2) << std::endl;
			std::cout << std::endl << "FTl.x: " << F_TailL(0) << " FTl.y: " << F_TailL(1) << " FTl.z: " << F_TailL(2) << std::endl;
//							  		
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
	
	Eigen::MatrixXd Aerodinamica::SkewSymmetricMatrix(Eigen::VectorXd Vector)
	{
		//Place Vet in the Skew Symmetric matrix S
		Eigen::MatrixXd SkewMatrix(3,3);
		SkewMatrix <<            0,      -Vector(2),       Vector(1),
				 Vector(2),               0,      -Vector(0),
				-Vector(1),       Vector(0),               0;
	      
		return SkewMatrix;
	}



	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	
	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}
				
