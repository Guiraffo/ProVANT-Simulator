/*
* File: Aerodinamica.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 09/04/2019
* Description:  This library is responsable to implement the aerodynamic forces of Fuselage, Wings and Tail surfaces on the UAV 4.0.The work was based on Daniel Neri phd thesis.
*/				




#include <Aerodinamica.h>
#include "/usr/include/eigen3/Eigen/Eigen"
//#include "SkewSymmetricMatrix.h"

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica::Aerodinamica(): DBf(3), DBwr(3), DBwl(3), DBtr(3), DBtl(3), RI_B(3,3), Wn(3,3), WI_IB(3), PhipThetapPsip(3), XpYpZp(3), EnvironmentWind(3), RBmiL(3,3), RBmiR(3,3), RBwdL(3,3), RBwdR(3,3), dPI_f(3), dPI_wr(3), dPI_wl(3), dPI_tr(3), dPI_tl(3), UVWf(3), UVWfa(3), UVWwr(3), UVWwra(3), UVWwl(3), UVWwla(3), UVWtr(3), UVWtra(3), UVWtl(3), UVWtla(3), RAlphaf(3,3), RBetaf(3,3), 
	Fxz(3), Fxy(3), Forca_F(3), FWrxz(3), FWlxz(3), Forca_Wr(3), Forca_Wl(3), RAlphawr(3,3), RAlphawl(3,3), RGammatr(3,3), RGammatl(3,3), Forca_Tr(3), Forca_Tl(3), F_TailR(3), F_TailL(3)
	{
		rho = 1.21;	//densidade do ar
		//sf = 2.0*0.0946;    //area da superficie da fuselagem
		SfFrontal = 2.0*0.0946;
		SfLateral = 2.2*0.1323;
		sw = 2.0*0.1125;   //area da superficie da asa
	 	st = 2.0*0.0494;  //area da superficie do Rudder esquerdo
        	T = 0.0001;	
        	mi = 0.5236; // tail deflection rad
        	wd = 0.0873; // wing diedral rad
				
		//std::string relativeFile("Wind.txt");
		//std::string file = std::getenv("TILT_MATLAB") + relativeFile;
		//Wind.startFile(file,"wind");
		
					//Environment wind-speed w.r.t the Inertial Frame
		EnvironmentWind << 0, 0, 0;
		
		//Position of the Aerodinamic centers w.r.t the Body frame expressed in Body frame
		Eigen::VectorXd PosCG(3);
		PosCG << 0.06684, 0, 0.005392;
		DBf  << 0.0512, 0, 0.1; // Position aerodynamic center of fuselage
		DBwr << 0.0512, -0.31, 0.1; // Position aerodynamic center of wing R
		DBwl << 0.0512,  0.31, 0.1; // Position aerodynamic center of wing L
		DBtr << -0.3967, -0.168, 0.148; // Position aerodynamic center of tail R
		DBtl << -0.3967,  0.168, 0.148; // Position aerodynamic center of tail L
		
		DBf  =  DBf - PosCG;
		DBwr = DBwr - PosCG;	
		DBwl = DBwl - PosCG;
		DBtr = DBtr - PosCG;	
		DBtl = DBtl - PosCG;	
				
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
			
			
			//modificado para melhor desempenho de aplicação de força dos propellers
			NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL",_sdf);
			juntaL = _model->GetJoint(NameOfJointL_);
			NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR",_sdf);
			juntaR = _model->GetJoint(NameOfJointR_);

		
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
		{	//First try
			math::Vector3 forceR(0,0,msg.data);
			linkFr->AddRelativeForce(forceR);
			
			
			//Second try
//			Eigen::VectorXd Fr(3),FrB(3);
//			Eigen::MatrixXd RBsR(3,3);
//			double AlphaR = juntaR->GetAngle(0).Radian();
//			double B = 0.0873;

//			RBsR << cos(AlphaR), -sin(AlphaR)*sin(B), cos(B)*sin(AlphaR),
//			           	  0,             cos(B),            sin(B),
//			       -sin(AlphaR), -cos(AlphaR)*sin(B), cos(AlphaR)*cos(B);
//			Fr << 0, 0, msg.data;
//			FrB = RBsR * Fr;
//			math::Vector3 forceR(FrB(0),FrB(1),FrB(2));
//			linkFr->AddRelativeForce(forceR);

			//Third try
//			//Computing the wind properties
//			math::Pose pose = link->GetWorldPose();
//			
//			//computing configuration variables
//			Phi = pose.rot.GetAsEuler( ).x;
//			Theta = pose.rot.GetAsEuler( ).y;
//			Psi = pose.rot.GetAsEuler( ).z;
//			
//			//computing transformation matrices
//			RI_B <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),(cos(Theta)*sin(Psi)), 
//	                         (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
//        	                 (-sin(Theta)), (cos(Theta)*sin(Phi)), (cos(Phi)*cos(Theta));


//			Eigen::MatrixXd RBsR(3,3);
//			double AlphaR = juntaR->GetAngle(0).Radian();
//			double B = 0.0873;

//			RBsR << cos(AlphaR), -sin(AlphaR)*sin(B), cos(B)*sin(AlphaR),
//			           	  0,             cos(B),            sin(B),
//			       -sin(AlphaR), -cos(AlphaR)*sin(B), cos(AlphaR)*cos(B);
//			Eigen::VectorXd Fr(3),FrI(3);
//			Fr << 0, 0, msg.data;
//			FrI = RI_B * RBsR * Fr;
//			linkFr->AddForceAtRelativePosition( math::Vector3( FrI(0),  FrI(1),  FrI(2)), math::Vector3( 0, 0, 0.0));
//			
			math::Vector3 torqueR(0,0,0.0178947368*msg.data); // drag torque
			// Applying			
			linkFr->AddRelativeTorque(torqueR);
			
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
			//First try
			math::Vector3 forceL(0,0,msg.data);
			linkFl->AddRelativeForce(forceL);	
			
			
			//second try
//			Eigen::VectorXd Fl(3),FlB(3);
//			Eigen::MatrixXd RBsL(3,3);
//			double AlphaL = juntaL->GetAngle(0).Radian();
//			//std::cout << AlphaL << std::endl;
//			double B = 0.0873;
//			RBsL << cos(AlphaL), sin(AlphaL)*sin(B), cos(B)*sin(AlphaL),
//			           	  0,             cos(B),            -sin(B),
//			       -sin(AlphaL), cos(AlphaL)*sin(B), cos(AlphaL)*cos(B);
//			Fl << 0, 0, msg.data;
//			FlB = RBsL * Fl;
//			math::Vector3 forceL(FlB(0),FlB(1),FlB(2));
//			linkFl->AddRelativeForce(forceL);
//			
			
			//Third try
//			//Computing the wind properties
//			math::Pose pose = link->GetWorldPose();
//			
//			//computing configuration variables
//			Phi = pose.rot.GetAsEuler( ).x;
//			Theta = pose.rot.GetAsEuler( ).y;
//			Psi = pose.rot.GetAsEuler( ).z;
//			
			//computing transformation matrices
//			RI_B <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),(cos(Theta)*sin(Psi)), 
//	                         (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
//        	                 (-sin(Theta)), (cos(Theta)*sin(Phi)), (cos(Phi)*cos(Theta));
//			
//			Eigen::MatrixXd RBsL(3,3);
//			double AlphaL = juntaL->GetAngle(0).Radian();
//			//std::cout << AlphaL << std::endl;
//			double B = 0.0873;
//			RBsL << cos(AlphaL), sin(AlphaL)*sin(B), cos(B)*sin(AlphaL),
//			           	  0,             cos(B),            -sin(B),
//			       -sin(AlphaL), cos(AlphaL)*sin(B), cos(AlphaL)*cos(B);
//			Eigen::VectorXd Fl(3),FlI(3);
//			Fl << 0, 0, msg.data;
//			FlI = RI_B * RBsL * Fl;
//			linkFl->AddForceAtRelativePosition( math::Vector3( FlI(0),  FlI(1),  FlI(2)), math::Vector3( 0, 0, 0.0));

			math::Vector3 torqueL(0,0,-0.0178947368*msg.data); // drag torque
			// Applying			
			linkFl->AddRelativeTorque(torqueL);
			
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
		
			AileronRDeflection = DaR.data;
			//std::cout << "AileronR: " << AileronRDeflection << std::endl;
			
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
			AileronLDeflection = DaL.data;
			//std::cout << "AileronL: " << AileronLDeflection << std::endl;
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
			RudderRDeflection = DrR.data;
			//std::cout << "RudderR: " << RudderRDeflection << std::endl; 
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
			RudderLDeflection = DrL.data;
			//std::cout << "RudderL: " << RudderLDeflection << std::endl;
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
			
			//computing configuration variables
			Phi = pose.rot.GetAsEuler( ).x;
			Theta = pose.rot.GetAsEuler( ).y;
			Psi = pose.rot.GetAsEuler( ).z;
			
			//computing transformation matrices
			RI_B <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),(cos(Theta)*sin(Psi)), 
				(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)), (cos(Theta)*sin(Phi)), (cos(Phi)*cos(Theta));
			Wn << 1.0,         0.0,          -sin(Theta), 
			       0.0,  cos(Phi),  cos(Theta)*sin(Phi),
	  	               0.0, -sin(Phi),  cos(Phi)*cos(Theta);
	  	               
       			//Diedral - Tail R and L - Rotation matrices in x-axis - mi = tail deflection
			RBmiL  <<  1,       0,        0,
				   0, cos(mi), -sin(mi),
				   0, sin(mi),  cos(mi);
			     
			RBmiR <<  1,        0,         0,
				  0, cos(-mi), -sin(-mi),
				  0, sin(-mi),  cos(-mi);
				 
			//Diedral - Wings R and L w.r.t x-axis - wd = wing deflection
			RBwdL <<  1,       0,        0,
				  0, cos(wd), -sin(wd),
				  0, sin(wd),  cos(wd);
			     
			RBwdR <<  1,        0,         0,
				  0, cos(-wd), -sin(-wd),
				  0, sin(-wd),  cos(-wd);
			
			//-----------Computing [phidot thetadot psidot]-----------------------%
			WI_IB << Angular.x, Angular.y, Angular.z;
			PhipThetapPsip = Wn.inverse() * RI_B.transpose() * WI_IB;
			
			//-----------Computing [Xdot Ydot Zdot]-------------------------------%
			XpYpZp << Linear.x, Linear.y, Linear.z;
			
			//Compute the velocity of the aerodynamic centers expressed in the Inertial frame
			dPI_f  << -RI_B*SkewSymmetricMatrix( DBf  )*Wn*PhipThetapPsip + XpYpZp; //velocity of the aerodynamic center of fuselage w.r.t I expressed in I
			dPI_wr << -RI_B*SkewSymmetricMatrix( DBwr )*Wn*PhipThetapPsip + XpYpZp; //velocity of the aerodynamic center of wing right w.r.t I expressed in I
			dPI_wl << -RI_B*SkewSymmetricMatrix( DBwl )*Wn*PhipThetapPsip + XpYpZp; //velocity of the aerodynamic center of wing left w.r.t I expressed in I
			dPI_tr << -RI_B*SkewSymmetricMatrix( DBtr )*Wn*PhipThetapPsip + XpYpZp; //velocity of the aerodynamic center of tail right w.r.t I expressed in I
			dPI_tl << -RI_B*SkewSymmetricMatrix( DBtl )*Wn*PhipThetapPsip + XpYpZp; //velocity of the aerodynamic center of tail left w.r.t I expressed in I
			
			//----------Computing Properties of Relative wind for fuselage---------%

			UVWf  = RI_B.transpose() * dPI_f; //Express the velocity on the frame positioned at the aerodynamic center of fuselage
			UVWfa = RI_B.transpose() * EnvironmentWind; //Express the enviroment wind-speed on the frame positioned at the aerodynamic center of fuselage

			double Vfxz = pow(pow(UVWf(2)-UVWfa(2),2) + pow(UVWf(0)-UVWfa(0),2) , 0.5); //Magnitude x-z axis
			double Vfxy = pow(pow(UVWf(1)-UVWfa(1),2) + pow(UVWf(0)-UVWfa(0),2) , 0.5); //Magnitude x-y axis

			double Alphaf = atan2(UVWf(2)-UVWfa(2),UVWf(0)-UVWfa(0)); //Orientation - Angle of attack fuselage
			double Betaf =  atan2(UVWf(1)-UVWfa(1),UVWf(0)-UVWfa(0)); //Orientation - Side slip angle fuselage
			
			
			//plot everything in order to evaluate the results
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << std::endl<< std::endl<< std::endl<< std::endl;
			std::cout << std::endl << "Vfxz: " << Vfxz << "  Vfxy: " << Vfxy << std::endl;
			std::cout << "Alphaf: " << Alphaf << "  Betaf: " << Betaf << std::endl;
			
			//----------Computing Properties of Relative wind for wings---------%

			//Wing right
			UVWwr = RBwdR.transpose() * RI_B.transpose() * dPI_wr; //Express the velocity on the frame positioned at the aerodynamic center of wing R
			UVWwra = RBwdR.transpose() * RI_B.transpose() * EnvironmentWind; //Express the enviroment wind-speed on the frame positioned at the aerodynamic center of wing R
			double Vwr = pow( pow(UVWwr(2)-UVWwra(2) , 2) + pow(UVWwr(0)-UVWwra(0) , 2) , 0.5); //compute Magnitude
			double Alphawr = atan2( UVWwr(2)-UVWwra(2) , UVWwr(0)-UVWwra(0) ); //compute Orientation

			//Wing left
			UVWwl = RBwdL.transpose() * RI_B.transpose() * dPI_wl; //Express the velocity on the frame positioned at the aerodynamic center of wing L
			UVWwla = RBwdL.transpose() * RI_B.transpose() * EnvironmentWind; //Express the enviroment wind-speed on the frame positioned at the aerodynamic center of wing L
			double Vwl = pow( pow(UVWwl(2)-UVWwla(2),2) + pow(UVWwl(0)-UVWwla(0),2), 0.5); //compute Magnitude
			double Alphawl = atan2(UVWwl(2)-UVWwla(2), UVWwl(0)-UVWwla(0)); //compute Orientation
			
			//plot everything in order to evaluate the results
			std::cout << std::endl << "Vwr: " << Vwr << "  Vwl: " << Vwl << std::endl;
			std::cout << "Alphawr: " << Alphawr << "  Alphawl: " << Alphawl << std::endl;

//			//----------Computing Properties of Relative wind for V-tail---------%

			//Tail right
			UVWtr = RBmiR.transpose() * RI_B.transpose() * dPI_tr; //Express the velocity on the frame positioned at the aerodynamic center of Tail R
			UVWtra = RBmiR.transpose() * RI_B.transpose() * EnvironmentWind; //Express the enviroment wind-speed on the frame positioned at the aerodynamic center of wing R
			double Vtr = pow( pow(UVWtr(2)-UVWtra(2),2) + pow(UVWtr(0)-UVWtra(0),2), 0.5); //compute Magnitude
			double Gammatr = atan2( UVWtr(2)-UVWtra(2) , UVWtr(0)-UVWtra(0) ); //compute Orientation

			//Tail left
			UVWtl = RBmiL.transpose() * RI_B.transpose() * dPI_tl; //Express the velocity on the frame positioned at the aerodynamic center of Tail L
			UVWtla = RBmiL.transpose() * RI_B.transpose() * EnvironmentWind; //Express the enviroment wind-speed on the frame positioned at the aerodynamic center of wing L
			double Vtl = pow( pow(UVWtl(2)-UVWtla(2),2) + pow(UVWtl(0)-UVWtla(0),2) , 0.5); //compute Magnitude
			double Gammatl = atan2( UVWtl(2)-UVWtla(2) , UVWtl(0)-UVWtla(0) );  //compute Orientation
			
			//plot everything in order to evaluate the results
			std::cout << std::endl << "Vtr: " << Vtr << "  Vtl: " << Vtl << std::endl;
			std::cout << "Gammatr: " << Gammatr << "  Gammatl: " << Gammatl << std::endl;
			
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			
			//The vector of aerodynamic coefficients CD and CL are represented from -180 to 180 with data for each 0.1 rad/s
			//calculo dos coef. da fuselagem
			double Val = ((-Alphaf+3.1416)/0.1); //start by computing the index of the coefficient
			int Index = floor(Val); //round it to the floor value
			double Proporcao = Val - Index; // compute the proportion between the floor value and the next one
			
			double CDfa = VetCDf[Index] + Proporcao * (VetCDf[Index+1] - VetCDf[Index]); // make a linear interpolation between the floor value and the next one
			double CLfa = VetCLf[Index] + Proporcao * (VetCLf[Index+1] - VetCLf[Index]); // make a linear interpolation between the floor value and the next one
			CDfa = CDfa + 0.1;
			
			
			Val = ((-Betaf+3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;

			double CDfs = VetCDf[Index] + Proporcao * (VetCDf[Index+1] - VetCDf[Index]);
			double CLfs = VetCLf[Index] + Proporcao * (VetCLf[Index+1] - VetCLf[Index]);
			CDfs = CDfs + 0.1;
			
			
			//calculo dos coef. da asa
			Val = ((-Alphawr+3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;
			
			double CDWr  = VetCDW[Index] + Proporcao * (VetCDW[Index+1] - VetCDW[Index]);
			double CLWr  = VetCLW[Index] + Proporcao * (VetCLW[Index+1] - VetCLW[Index]);

			
			Val = ((-Alphawl+3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;
			
			double CDWl  = VetCDW[Index] + Proporcao * (VetCDW[Index+1] - VetCDW[Index]);
			double CLWl  = VetCLW[Index] + Proporcao * (VetCLW[Index+1] - VetCLW[Index]);	
			

			//Calculo dos coeficientes da Cauda	   
			Val = ((-Gammatr + 3.1416)/0.1);   
			Index = floor(Val);
			Proporcao = Val - Index;
			
			double CDTr  = VetCDt[Index] + Proporcao * (VetCDt[Index+1] - VetCDt[Index]);
			double CLTr  = VetCLt[Index] + Proporcao * (VetCLt[Index+1] - VetCLt[Index]);	
			
			
			Val = ((-Gammatl + 3.1416)/0.1);
			Index = floor(Val);
			Proporcao = Val - Index;	   
			
			double CDTl  = VetCDt[Index] + Proporcao * (VetCDt[Index+1] - VetCDt[Index]);
			double CLTl  = VetCLt[Index] + Proporcao * (VetCLt[Index+1] - VetCLt[Index]);
			
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			

			// aerodynamic forces actuating on the fuselagem
			double Fd_xz_F = -0.5 * rho * (Vfxz*Vfxz) * SfFrontal * CDfa;	
			double Fl_xz_F =  0.5 * rho * (Vfxz*Vfxz) * SfFrontal * CLfa;
			double Fd_xy_F = -0.5 * rho * (Vfxy*Vfxy) * SfLateral * CDfs;
			double Fl_xy_F =  0.5 * rho * (Vfxy*Vfxy) * SfLateral * CLfs;
			
			//Rotation matrices that map from wind to the aerodynamic center frame
			RAlphaf << cos(-Alphaf),  0, sin(-Alphaf),
					     0,   1,            0,
				  -sin(-Alphaf),  0, cos(-Alphaf);
				 
			RBetaf <<  cos(-Betaf), -sin(-Betaf),   0,
				   sin(-Betaf),  cos(-Betaf),   0,
				             0,            0,   1;
				
			//Make the vector of aerodynamic forces applied by the fuselage
			Fxz << Fd_xz_F, 0, Fl_xz_F;
			Fxy << Fd_xy_F, Fl_xy_F, 0;	
			
			//map the forces from the wind frame to the body frame		
			Forca_F = RI_B * (RAlphaf * Fxz + RBetaf * Fxy);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			
			//Aerodynamic forces applied by the wings	
		    	double Fd_Wr = -0.5 * rho * (Vwr * Vwr) * sw *  CDWr;
		    	double Fl_Wr =  0.5 * rho * (Vwr * Vwr) * sw * (CLWr + c_AileronR(AileronRDeflection));
		    	double Fd_Wl = -0.5 * rho * (Vwl * Vwl) * sw *  CDWl;
		    	double Fl_Wl =  0.5 * rho * (Vwl * Vwl) * sw * (CLWl + c_AileronL(AileronLDeflection));
			
			//Rotation matrices that map from wind to the aerodynamic center frame
			RAlphawr <<	cos(-Alphawr),  0  , sin(-Alphawr),
					          0  ,  1  ,            0,
				       -sin(-Alphawr),  0  , cos(-Alphawr);
			
			RAlphawl <<     cos(-Alphawl),  0  , sin(-Alphawl),
					          0  ,  1  ,            0,
				       -sin(-Alphawl),  0  , cos(-Alphawl);			
			
			//Make the vector of aerodynamic forces applied by the fuselage
			FWrxz << Fd_Wr, 0, Fl_Wr;
			FWlxz << Fd_Wl, 0, Fl_Wl;
			
			//map the forces from the wind frame to the body frame	
			Forca_Wr = RI_B * RBwdR * RAlphawr * FWrxz; 
			Forca_Wl = RI_B * RBwdL * RAlphawl * FWlxz;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			   
			
			//Aerodynamic forces applied by the V-tail surfaces	
			double Fd_TR = -0.5 * rho * (Vtr * Vtr) * st * CDTr;
			double Fl_TR =  0.5 * rho * (Vtr * Vtr) * st * (CLTr + c_RudR(RudderRDeflection));
			double Fd_TL = -0.5 * rho * (Vtl * Vtl) * st * CDTl;
			double Fl_TL =  0.5 * rho * (Vtl * Vtl) * st * (CLTl + c_RudL(RudderLDeflection));
			
			//Rotation matrices that map from wind to the aerodynamic center frame
			RGammatr << cos(-Gammatr) , 0 , sin(-Gammatr),
					        0 , 1 ,            0,
			           -sin(-Gammatr) , 0 , cos(-Gammatr);
			            
			            
			RGammatl <<  cos(-Gammatl), 0 , sin(-Gammatl),
					         0, 1 ,            0,
			            -sin(-Gammatl), 0 , cos(-Gammatl);
			
			//Make the vector of aerodynamic forces applied by the V-tail surfaces
			F_TailR  << Fd_TR, 0, Fl_TR;
			F_TailL  << Fd_TL, 0, Fl_TL;
			
			//map the forces from the wind frame to the body frame	
			Forca_Tr = RI_B * RBmiR * RGammatr * F_TailR;
			Forca_Tl = RI_B * RBmiL * RGammatl * F_TailL;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			   
			
			
			//Apply to gazebo
			MainBody->AddForceAtRelativePosition( math::Vector3( Forca_F(0),  Forca_F(1),  Forca_F(2)), math::Vector3(  DBf(0),  DBf(1),  DBf(2)));
			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_Wr(0), Forca_Wr(1), Forca_Wr(2)), math::Vector3( DBwr(0), DBwr(1), DBwr(2)));
      			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_Wl(0), Forca_Wl(1), Forca_Wl(2)), math::Vector3( DBwl(0), DBwl(1), DBwl(2)));
			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_Tr(0), Forca_Tr(1), Forca_Tr(2)), math::Vector3( DBtr(0), DBtr(1), DBtr(2)));
      			MainBody->AddForceAtRelativePosition( math::Vector3(Forca_Tl(0), Forca_Tl(1), Forca_Tl(2)), math::Vector3( DBtl(0), DBtl(1), DBtl(2)));
			
			//plot everything in order to evaluate the results]
			//std::cout << std::endl << "CaR:" << c_AileronR(ElevatorDeflectionR) << "CaL:" << c_AileronL(ElevatorDeflectionL) << "CrR:" << c_RudR(RudderDeflectionR) << "CrL:" << c_RudL(RudderDeflectionL) << std::endl;
			//std::cout << std::endl << "RudderR: " << RudderDeflectionR << " RudderL: " << RudderDeflectionL << std::endl;	      
			std::cout << std::endl << "Ff.x:  " <<  Forca_F(0)  << " Ff.y: "  <<  Forca_F(1)  << " Ff.z:  " <<  Forca_F(2) << std::endl;	
			std::cout << std::endl << "FWr.x: " << Forca_Wr(0)  << " FWr.y: " << Forca_Wr(1)  << " FWr.z: " << Forca_Wr(2) << std::endl;
			std::cout << std::endl << "FWl.x: " << Forca_Wl(0)  << " FWl.y: " << Forca_Wl(1)  << " FWl.z: " << Forca_Wl(2) << std::endl;
			std::cout << std::endl << "FTr.x: " << Forca_Tr(0)  << " FTr.y: " << Forca_Tr(1)  << " FTr.z: " << Forca_Tr(2) << std::endl;
			std::cout << std::endl << "FTl.x: " << Forca_Tl(0)  << " FTl.y: " << Forca_Tl(1)  << " FTl.z: " << Forca_Tl(2) << std::endl;
			std::cout << "AileronR: " << AileronRDeflection << "AileronL: " << AileronLDeflection << "RudderR: " << RudderRDeflection << "RudderL: " << RudderLDeflection << std::endl;							  		
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
			return 0.511*DeR;
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
			return 0.511*DeL;
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
			return 0.85*DrR;
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
			return 0.85*DrL;
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
				
