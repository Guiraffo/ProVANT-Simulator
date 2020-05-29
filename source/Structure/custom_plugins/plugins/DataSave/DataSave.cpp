#include <DataSave.h>

namespace gazebo
{

	//Constructor
	DataSave::DataSave() : PhipThetapPsip(3,1), RIB(3,3), W_n(3,3), WIIB(3,1), q(8), qp(8), u(8), qpBody(8), XpYpZp(3,1)
	{

	}
  //Destructor
	DataSave::~DataSave()
	{	
		try
		{
			GeneralizedCoordinatesNVelocities.endFile();
			updateTimer.Disconnect(updateConnection);
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		} 
	}	

/*********************************************************************************************************************************************/

	
	void DataSave::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	
	{	
	
		
		try
		{
	  		if (!ros::isInitialized())
	  		{
	    			std::cout << "DataSave is not initialized!" << std::endl;
	    		        return;
	  		}
	  		else{
  			std::cout<< "Savedata plugin initialized!" <<std::endl;
				this->model = _parent;// Store the pointer to the model
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DataSave::OnUpdate, this)); // Listen to the update event. This event is broadcast every
         			
        // update timer
        world = _parent->GetWorld();
			  Reset();
			  updateTimer.Load(world, _sdf);
				updateConnection = updateTimer.Connect(boost::bind(&DataSave::OnUpdate, this));
				
				
				NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // Get name of topic to publish data
				NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR",_sdf); // name of the right joint
				NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL",_sdf); // name of the left joint
			
				world = model->GetWorld();	// pointer to the world
				juntaR = model->GetJoint(NameOfJointR_); // pointer to the right joint
				juntaL = model->GetJoint(NameOfJointL_); // pointer to the left joint

				link_name_ = XMLRead::ReadXMLString("bodyName",_sdf); // name of the main body
				link = model->GetLink(link_name_); // pointer to the main body			

				//open file to save data
				std::string RelativeFile1("GeneralizedCoordinatesNVelocities.txt");
				std::string GeneralizedCoordinatesNVelocitiesFile = std::getenv("TILT_MATLAB") + RelativeFile1;
				GeneralizedCoordinatesNVelocities.startFile(GeneralizedCoordinatesNVelocitiesFile,"GeneralizedCoordinatesNVelocities");
				
				std::string RelativeFile2("GeneralizedVelocitiesExpressedOnbodyFrame.txt");
				std::string GeneralizedVelocitiesExpressedOnbodyFrameFile = std::getenv("TILT_MATLAB") + RelativeFile2;
				GeneralizedVelocitiesExpressedOnbodyFrame.startFile(GeneralizedVelocitiesExpressedOnbodyFrameFile,"GeneralizedVelocitiesExpressedOnbodyFrame");

		
	  		}
						
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}


/**********************************************************************************************************************************************/
	void DataSave::Reset()
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
/**********************************************************************************************************************************************/
	void DataSave::OnUpdate()
	{
		try
		{	
			
			ignition::math::Pose3d pose = link->WorldPose();
			
			//compute generalized coordinates
			q << pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), pose.Rot().Euler().X(), pose.Rot().Euler().Y(), pose.Rot().Euler().Z(), juntaR->Position(0), juntaL->Position(0);

			
			//compute the generalized velocities
			ignition::math::Vector3d linear = link->WorldLinearVel();
			ignition::math::Vector3d angular = link->WorldAngularVel( );
			
			Phi = pose.Rot().Euler().X();
			Theta = pose.Rot().Euler().Y();
			Psi = pose.Rot().Euler().Z();
			
			RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
        	                (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));
        	                
			W_n << 1.0,         0.0,          -sin(Theta), 
			       0.0,  cos(Phi),  cos(Theta)*sin(Phi),
	  	               0.0, -sin(Phi),  cos(Phi)*cos(Theta);
	  	               
			//Get the angular velocity w.r.t I expressed in I and maps to obtain the time derivative of Euler angles
			WIIB << angular.X(), angular.Y(), angular.Z();
			PhipThetapPsip = W_n.inverse() * RIB.transpose() * WIIB;
			XpYpZp << linear.X(), linear.Y(), linear.Z();
			
			
			qp << linear.X(), linear.Y(), linear.Z(), PhipThetapPsip(0), PhipThetapPsip(1), PhipThetapPsip(2), juntaR->GetVelocity(0), juntaL->GetVelocity(0);
			
			
			//save generalized coordinates and velocities data
			std::vector<double> qNqp;
			qNqp.push_back(q(0));
			qNqp.push_back(q(1));
			qNqp.push_back(q(2));
			qNqp.push_back(q(3));
			qNqp.push_back(q(4));
			qNqp.push_back(q(5));
			qNqp.push_back(q(6));
			qNqp.push_back(q(7));
			qNqp.push_back(qp(0));
			qNqp.push_back(qp(1));
			qNqp.push_back(qp(2));
			qNqp.push_back(qp(3));
			qNqp.push_back(qp(4));
			qNqp.push_back(qp(5));
			qNqp.push_back(qp(6));
			qNqp.push_back(qp(7));

			
			static int Contador = 0;
			if(Contador%120 == 0){
				GeneralizedCoordinatesNVelocities.printFile(qNqp);
			}
			Contador = Contador++;
			
			
			//Compute the vector of generalized velocities expressed on the body frame
			UVW = RIB.transpose()*XpYpZp;
			PQR = W_n * PhipThetapPsip;
			qpBody << UVW(0), UVW(1), UVW(2), PQR(0), PQR(1), PQR(2), juntaR->GetVelocity(0), juntaL->GetVelocity(0);
			
			
						
			//save generalized velocities data expressed on the body frame
			std::vector<double> qpBodySave;
			qpBodySave.push_back(qpBody(0));
			qpBodySave.push_back(qpBody(1));
			qpBodySave.push_back(qpBody(2));
			qpBodySave.push_back(qpBody(3));
			qpBodySave.push_back(qpBody(4));
			qpBodySave.push_back(qpBody(5));
			qpBodySave.push_back(qpBody(6));
			qpBodySave.push_back(qpBody(7));
			static int Contador2 = 0;
			if(Contador%120 == 0){
				GeneralizedVelocitiesExpressedOnbodyFrame.printFile(qpBodySave);
			}
			Contador2 = Contador2++;
			
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}	
	
/**********************************************************************************************************************************************/		
	GZ_REGISTER_MODEL_PLUGIN(DataSave)
}
