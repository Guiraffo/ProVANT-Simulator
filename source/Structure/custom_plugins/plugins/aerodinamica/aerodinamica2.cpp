/*! \brief Brief description.
 *         Brief description continued.
 *
 *  Detailed description starts here.
 */

#include <aerodinamica2.h>

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica2::Aerodinamica2() //:rho(1.21),sf(0.2072),sh(0.0118),sv(0.0079),kt(1.7e-7),b(9.5e-6)
	{
		rho = 1.21;	//densidade do ar
		sf = 0.123758; //area da superficie da fuselagem
		sh = 0.026482; //area da superficie do estabilizador horizontal
		sv = 0.0528758; //area da superficie do estabilizador vertical
	//	kt = 1.7e-7;
	//	b = 9.5e-6;
		
		R_IB.setZero(3,3);
		wind_B.setZero(3,1); //vento no corpo
		wind_I.setZero(3,1); //vento inercial
		wind_I(0) = 0;
	}

	Aerodinamica2::~Aerodinamica2()
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
		  
		  	// Vento
			pose = link->GetWorldPose();	
			phi = pose.rot.GetAsEuler().x;
			theta = pose.rot.GetAsEuler().y;
			psi = pose.rot.GetAsEuler().z;	
			R_IB << (cos(psi)*cos(theta)), (cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi)), (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)),
				(cos(theta)*sin(psi)), (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)),
				(-sin(theta)), (cos(theta)*sin(phi)), (cos(phi)*cos(theta));
			wind_B = R_IB.transpose() * wind_I;

			ua = wind_B(0,0);
			va = wind_B(1,0);
			wa = wind_B(2,0);

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
			//JointFr->SetForce(0,(kt/b)*(msg.data));
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
			//JointFl->SetForce(0,-(kt/b)*(msg.data));
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
			// Atualiza as velocidades lineares do corpo:
			math::Vector3 linear = link->GetRelativeLinearVel();				
			ub = linear.x; 
			vb = linear.y;
			wb = linear.z; 
			
			alpha = getAlpha(wb,wa,ub,ua);
			//velocidade do vento
			air_xz = getAirxz(vb,va,ub,ua);
			//eleva velocidade do vento ao quadrado
			v_xz = pow(air_xz,2);
			
			Fh.x = (0.5)*rho*sh*v_xz*( ( cl_hxz(alpha) + c_e(De.data) )*sin(alpha) - cd_hxz(alpha)*cos(alpha) );
			Fh.y = 0;
			Fh.z = (0.5)*rho*sh*v_xz*( cd_hxz(alpha)*sin(alpha) + ( cl_hxz(alpha) + c_e(De.data) )*cos(alpha) );

			// Aplica no estabilizador horizontal (elevator):
			linkE->AddRelativeForce(Fh);		
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
			// Atualiza as velocidades lineares do corpo:
			math::Vector3 linear = link->GetRelativeLinearVel();				
			ub = linear.x; 
			vb = linear.y;
			wb = linear.z; 
			
			beta = getBeta(vb,va,ub,ua);
			//velocidade do vento
			air_xy = getAirxy(wb,wa,ub,ua);
			//eleva velocidade do vento ao quadrado
			v_xy = pow(air_xy,2);

			Fv.x = (0.5)*rho*sv*v_xy*( ( cl_vxy(beta) + c_r(Dr.data) )*sin(beta) - cd_vxy(beta)*cos(beta) );
			Fv.y = (0.5)*rho*sv*v_xy*( cd_vxy(beta)*sin(beta) + ( cl_vxy(beta) + c_r(Dr.data) )*cos(beta) );
			Fv.z = 0;

			// Aplica no estabilizador horizontal (rudder):
			linkR->AddRelativeForce(Fv);			
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
			common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);

			// Atualiza as velocidades lineares do corpo:
			math::Vector3 linear = link->GetRelativeLinearVel();				
			ub = linear.x; 
			vb = linear.y;
			wb = linear.z; 

			alpha = getAlpha(wb,wa,ub,ua);
			beta = getBeta(vb,va,ub,ua);	

			if(alpha > 0.4363)
			{
				alpha = 0.4363;
			}else if( alpha < -0.4363){
				alpha = -0.4363;
			}
			
			if(beta > 0.4363)
			{
				beta = 0.4363;
			}else if( beta < -0.4363){
				beta = -0.4363;
			}


			//velocidade do vento
			air_xy = getAirxy(wb,wa,ub,ua);
			air_xz = getAirxz(vb,va,ub,ua);

			//eleva velocidade do vento ao quadrado
			v_xy = pow(air_xy,2);
			v_xz = pow(air_xz,2);

			// Cálculo da força Ff:
			//double Ff_x, Ff_y, Ff_z;
			Ff.x = (0.5)*rho*sf*v_xz*( cl_fxz(alpha)*sin(alpha) - cd_fxz(alpha)*cos(alpha) ) +
				   (0.5)*rho*sf*v_xy*( cl_fxy(beta)*sin(beta) - cd_fxy(beta)*cos(beta) );

			Ff.y = (0.5)*rho*sf*v_xy*( cl_fxy(beta)*cos(beta) + cd_fxy(beta)*sin(beta) );

			Ff.z = (0.5)*rho*sf*v_xz*( cl_fxz(alpha)*cos(alpha) + cd_fxz(alpha)*sin(alpha));

			// Aplica no corpo:
			link->AddRelativeForce(Ff);
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
			return beta;
		}
		catch(std::exception& e)	
		{
			std::cout << e.what() << std::endl;
		}
	}

	double Aerodinamica2::getAirxy(double wb, double wa, double ub, double ua)
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

	double Aerodinamica2::getAirxz(double vb, double va, double ub, double ua)
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

	double Aerodinamica2::cd_fxz(double alpha)
	{
		try
		{
			return 0.4566*pow(alpha,2)-0.0403*alpha+0.0601;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	

	}
	
	double Aerodinamica2::cl_fxz(double alpha)
	{
		try
		{
			return 0.5405*alpha-0.0353;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}

	double Aerodinamica2::cd_fxy(double beta)
	{
		try
		{
			//return 0.3513*pow(beta,2)+0.0604;
			return 0.3513*pow(beta,2) -10e-15*beta +0.0604;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}

	double Aerodinamica2::cl_fxy(double beta)
	{
		try
		{
			return 0.3821*beta;//-0.003;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}

	double Aerodinamica2::cd_vxy(double beta)
	{
		try
		{
			//return 2.2019*pow(beta,2)+0.0149;
			return 2.2019*pow(beta,2) -2e-7*beta +0.0149;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}

	double Aerodinamica2::cl_vxy(double beta)
	{
		try
		{
			return -45.392*pow(beta,3)+0.0011*pow(beta,2)+6.1126*beta;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}	
	
	double Aerodinamica2::cd_hxz(double alpha)
	{
		try
		{
			return 1.9382*pow(alpha,2)+0.0088;
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 	
	}
	
	double Aerodinamica2::cl_hxz(double alpha)
	{
		try
		{
			//return -35.216*pow(alpha,3)+6.5360*alpha;
			return -35.216*pow(alpha,3)+9e-13*pow(alpha,2) +6.5360*alpha - 6e-14;
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
			return 2.1873375*Dr;
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
