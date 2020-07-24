#include <turbulance.h>

namespace gazebo
{

  turbulance::turbulance(): v(3),Ad(8,8),Bd(8,3),Cd(3,8)
  {
    const char* _pathconfig = "/home/jonatas/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/models/vant_4_aerod/config/config.xml";
		XMLRead2 docme(_pathconfig); // start read XML data at the address placed in an environment variable
		TurbTag = docme.GetItem("Turbulance").c_str();

    //Defines the Ad,Bd and Cd matrices for Von Karman turbulance ss model
Ad << -0.090289,-0.00087965,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,
       0,0,-0.084848,-0.00085059,-1.8769e-06,0,0,0,
       0,0,1,0,0,0,0,0,
       0,0,0,1,0,0,0,0,
       0,0,0,0,0,-0.64178,-0.048665,-0.00081222,
       0,0,0,0,0,1,0,0,
       0,0,0,0,0,0,1,0;
 	 Bd << 1,0,0,
          0,0,0,
          0,1,0,
          0,0,0,
          0,0,0,
          0,0,1,
          0,0,0,
          0,0,0;
   Cd << 2.2658,0.11982,0,0,0,0,0,0,
          0,0,1.9881,0.10627,0.00025566,0,0,0,
          0,0,0,0,0,0.16713,0.067574,0.0012296;
    delT = 0.012;
  }
  turbulance::~turbulance()
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

  void turbulance::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
      try
      {
    std::cout << "entrei Turbulance plugin"<<std::endl;
              if (!ros::isInitialized())
              {
            ROS_INFO("Nao inicializado!");
                        return;
              }
              world = _model->GetWorld();
              // notifying when occurs new step time
    	  		Reset();
    			  updateTimer.Load(world, _sdf);
    	  		updateConnection = updateTimer.Connect(boost::bind(&turbulance::Update, this));


          NameOfTopic_ = XMLRead::ReadXMLString("NameOfTopic",_sdf); // Get name of topic to publish data
          publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfTopic_, 1);
        }
          catch(std::exception& e)
          		{
          			std::cout << e.what() << std::endl;
          		}
  }

  // reset
void turbulance::Reset()
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

void turbulance::Update()
{
  try
  {
    boost::mutex::scoped_lock scoped_lock(lock); // mutex

    if (TurbTag == "Custom_Model")
      {
                //User Implementation
      }


    //-------------Implements Von Karman Turbulance Model-------------
    if (TurbTag == "Von_Karman")
    {
      std::cout << "Von Karman Model" << std::endl;
      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
     std::default_random_engine generator (seed);
     //Defines a normal distribution
      std::normal_distribution<double> distribution_x (0.0,0.1);
      std::normal_distribution<double> distribution_y (0.0,0.1);
      std::normal_distribution<double> distribution_z (0.0,0.1);

      v << distribution_x(generator), distribution_y(generator),distribution_z(generator);
  //    std::cout <<"NORMAL DISTRIB:" <<distribution_x(generator) <<std::endl;
  		static Eigen::VectorXd  dp_ant;
      dp_ant = Bd*v;

      static Eigen::VectorXd  d(8);
      d << 0, 0,0,0,0,0,0,0;
      static Eigen::VectorXd  dpint(8);
      dpint << 0,0,0,0,0,0,0,0;
      dp = Ad*d + Bd*v;

      dpint = dpint + (delT/2)*(dp + dp_ant);
      dp_ant = dp;
      d = dpint;

      EnvWind = Cd*dpint;
      std::cout << "TESTEenvwind" << EnvWind << std::endl;

      simulator_msgs::Sensor newmsg;
      newmsg.name = NameOfTopic_;
      newmsg.header.stamp = ros::Time::now(); // time stamp
      newmsg.header.frame_id = "1";
      newmsg.values.push_back(EnvWind(0)); // x
      newmsg.values.push_back(EnvWind(1)); // y
      newmsg.values.push_back(EnvWind(2)); // z

      publisher_.publish(newmsg);
  }

  if (TurbTag == "Turbulance Model 2")
  {

  }



}
  catch(std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

GZ_REGISTER_MODEL_PLUGIN(turbulance)
}
