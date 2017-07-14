#include "sonar.h"

#include "std_msgs/String.h"
#include <sstream>

namespace gazebo
{
	/////////////////////////////////////////////////
	sonar::sonar() 
	{
	}

	/////////////////////////////////////////////////
	sonar::~sonar()
	{

	}

	/////////////////////////////////////////////////
	void sonar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	  
		gazebotopic = XMLRead::ReadXMLString("gazebotopic",_sdf);
		rostopic = XMLRead::ReadXMLString("rostopic",_sdf);
	  	std::string link = XMLRead::ReadXMLString("link",_sdf);;
	  
      		this->model = _model;
      		this->node = transport::NodePtr(new transport::Node());
      		this->node->Init(this->model->GetWorld()->GetName());
      		std::string topic = "/gazebo/default/" + this->model->GetName() + "/" + link + "/" + gazebotopic;
      		this->sub = this->node->Subscribe(topic, &sonar::OnUpdate, this);
      		publisher_ = n.advertise<simulator_msgs::Sensor>(rostopic, 1);

	}

	/////////////////////////////////////////////////
	void sonar::OnUpdate(ConstSonarPtr &msg)
	{	 	
		simulator_msgs::Sensor newmsg;
		newmsg.name = rostopic;
		newmsg.header.stamp = ros::Time::now();
		newmsg.header.frame_id = "1";
		newmsg.values.push_back(msg->range());
		publisher_.publish(newmsg);
	}
	GZ_REGISTER_MODEL_PLUGIN(sonar)
}
