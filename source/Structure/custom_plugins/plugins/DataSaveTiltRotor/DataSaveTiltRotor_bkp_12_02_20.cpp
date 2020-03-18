/*
* File: QuadForces.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 10/12/19
* Description:  This library is responsable to implement aerodynamics forces in a UAV
*/

#include <DataSaveTiltRotor.h>

namespace gazebo
{
	// constructor
	DataSaveTiltRotor::DataSaveTiltRotor() 
	{
	 
	}
	// destructor
	DataSaveTiltRotor::~DataSaveTiltRotor()
	{	
		ControlInputsFile.endFile();
	}
	// to load initial setup
	void DataSaveTiltRotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		std::cout<< "DataSavetiltRotor Inicializado" <<std::endl;
		try
		{
	    		if (!ros::isInitialized())
	    		{
	      			std::cout << "DataSavetiltRotor not initialized!" << std::endl;
	      		        return;
	    		}
			
			topic_Fr = XMLRead::ReadXMLString("topic_Fr",_sdf); 
			topic_Fl = XMLRead::ReadXMLString("topic_Fl",_sdf); 
			topic_DAr = XMLRead::ReadXMLString("topic_DAr",_sdf); 
			topic_DAl = XMLRead::ReadXMLString("topic_DAl",_sdf); 
			topic_DRr = XMLRead::ReadXMLString("topic_DRr",_sdf); 
			topic_DRl = XMLRead::ReadXMLString("topic_DRl",_sdf); 
		  
		  Fr_sat = XMLRead::ReadXMLDouble("Fr_sat",_sdf); 
		  Fl_sat = XMLRead::ReadXMLDouble("Fl_sat",_sdf); 
		  DAr_sat = XMLRead::ReadXMLDouble("DAr_sat",_sdf); 
		  DAl_sat = XMLRead::ReadXMLDouble("DAl_sat",_sdf); 
		  DRr_sat = XMLRead::ReadXMLDouble("DRr_sat",_sdf); 
		  DRl_sat = XMLRead::ReadXMLDouble("DRl_sat",_sdf); 
	
			
			
			
			

			//open file to save data
				std::string RelativeFile1("ControlInputs.txt");
				std::string ControlInputs = std::getenv("TILT_MATLAB") + RelativeFile1;
				ControlInputsFile.startFile(ControlInputs,"ControlInputs");

			// update timer
	 // 		Reset();
	//			updateTimer.Load(world, _sdf);
	//		  updateConnection = updateTimer.Connect(boost::bind(&QuadForces::Update, this));
			// subscribers of data to apply in simulator			
			subscriberFr_ = node_handle_.subscribe(topic_Fr, 1, &gazebo::DataSaveTiltRotor::CallbackFr, this);
			subscriberFl_ = node_handle_.subscribe(topic_Fl, 1, &gazebo::DataSaveTiltRotor::CallbackFl, this);
			subscriberDAr_ = node_handle_.subscribe(topic_DAr, 1, &gazebo::DataSaveTiltRotor::CallbackDAr, this);
			subscriberDAl_ = node_handle_.subscribe(topic_DAl, 1, &gazebo::DataSaveTiltRotor::CallbackDAl, this);
			subscriberDRr_ = node_handle_.subscribe(topic_DRr, 1, &gazebo::DataSaveTiltRotor::CallbackDRr, this);
			subscriberDRl_ = node_handle_.subscribe(topic_DRl, 1, &gazebo::DataSaveTiltRotor::CallbackDRl, this);
		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	// when reset simulator
	void DataSaveTiltRotor::Reset()
	{
		try
		{
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// callback to apply forces at right brushless
	void DataSaveTiltRotor::CallbackFr(std_msgs::Float64 msg)
	{
		try
		{  	  
		 if(msg.data > Fr_sat){        
			Fr = Fr_sat;
			}else if(msg.data < -Fr_sat){
			Fr = -Fr_sat;
			}
			SaveTiltData("fr",Fr);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// callback to apply forces at left brushless
	void DataSaveTiltRotor::CallbackFl(std_msgs::Float64 msg)
	{
		try
		{	
			if(msg.data > Fl_sat){        
			Fl = Fl_sat;
			}else if(msg.data < -Fr_sat){
			Fl = -Fl_sat;
			}
			SaveTiltData("fl",Fl);	
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void DataSaveTiltRotor::CallbackDAr(std_msgs::Float64 msg)
	{
		try
		{
			if(msg.data > DAr_sat){        
			DAr = DAr_sat;
			}else if(msg.data < -DAr_sat){
			DAr = -DAr_sat;
			}
			SaveTiltData("DAr",DAr);		 
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void DataSaveTiltRotor::CallbackDAl(std_msgs::Float64 msg)
	{
		try
		{
			if(msg.data > DAl_sat){        
			DAl = DAl_sat;
			}else if(msg.data < -DAl_sat){
			DAl = -DAl_sat;
			}
			SaveTiltData("DAl",DAl);	
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DataSaveTiltRotor::CallbackDRr(std_msgs::Float64 msg)
	{
		try
		{
			if(msg.data > DRr_sat){        
			DRr = DRr_sat;
			}else if(msg.data < -DRr_sat){
			DRr = -DRr_sat;
			}
			SaveTiltData("DRr",DRr);	
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DataSaveTiltRotor::CallbackDRl(std_msgs::Float64 msg)
	{
		try
		{
			if(msg.data > DRl_sat){        
			DRl = DRl_sat;
			}else if(msg.data < -DRl_sat){
			DRl = -DRl_sat;
			}
			SaveTiltData("DRl",DRl);		
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DataSaveTiltRotor::SaveTiltData(std::string var,double val){

double fr,fl,Ar_d,Al_d,Rr_d,Rl_d;

if(var == "fr"){
fr = val;
}else if(var == "fl"){
fl = val;
}else if(var == "DAr"){
Ar_d = val;
}else if(var == "DAl"){
Al_d = val;
}else if(var == "DRr"){
Rr_d = val;
}else if(var =="DRl"){
Rl_d = val;
}

std::vector<double> DataVec;
DataVec.push_back(fr);
DataVec.push_back(fl);
DataVec.push_back(Ar_d);
DataVec.push_back(Al_d);
DataVec.push_back(Rr_d);
DataVec.push_back(Rl_d);

	static int Contador = 0;
			if(Contador%120 == 0){
				ControlInputsFile.printFile(DataVec);
			}
			Contador = Contador++;
}




	GZ_REGISTER_MODEL_PLUGIN(DataSaveTiltRotor)
}
