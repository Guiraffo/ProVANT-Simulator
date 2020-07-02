/*
* File: QuadForces.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/20
* Description:  This library is responsable to implement the rotational movement of the propellers
*/

#include <VisualPropellers.h>

namespace gazebo
{
	// constructor
	VisualPropellers::VisualPropellers() 
	{
	
	}
	// destructor
	VisualPropellers::~VisualPropellers()
	{	
		
	}
	// to load initial setup
	void VisualPropellers::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "VisualPropellers not initialized!" << std::endl;
	      		        return;
	    		}
			
			Propeller1_ = XMLRead::ReadXMLString("Propeller1",_sdf);
			Propeller2_ = XMLRead::ReadXMLString("Propeller2",_sdf);
			Velocity_ = XMLRead::ReadXMLDouble("Propellers_Velocity",_sdf);
			
			Propeller_1 = _model->GetJoint(Propeller1_);
			Propeller_2 = _model->GetJoint(Propeller2_);
			
			
			Propeller_1->SetVelocity(0,Velocity_);
			Propeller_2->SetVelocity(0,Velocity_);
			
			

		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// when reset simulator
	void VisualPropellers::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


	GZ_REGISTER_MODEL_PLUGIN(VisualPropellers)
}
