#include "magnetometer.h"

#include "std_msgs/String.h"
#include <sstream>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(magnetometer)

/////////////////////////////////////////////////
magnetometer::magnetometer() 
{

}

/////////////////////////////////////////////////
magnetometer::~magnetometer()
{
	//gazebo::client::shutdown();
}

/////////////////////////////////////////////////
void magnetometer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	
	gazebotopic = XMLRead::ReadXMLString("gazebotopic",_sdf);
	rostopic = XMLRead::ReadXMLString("rostopic",_sdf);
	std::string link = XMLRead::ReadXMLString("link",_sdf);

      this->model = _model;
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetWorld()->GetName());

      std::string topic = "/gazebo/default/" + this->model->GetName() + "/" + link + "/" + gazebotopic;
      this->sub = this->node->Subscribe(topic, &magnetometer::OnUpdate, this);
      publisher_ = n.advertise<simulator_msgs::Sensor>(rostopic, 1);

}

/////////////////////////////////////////////////
void magnetometer::OnUpdate(ConstMagnetometerPtr &_msg)
{
	simulator_msgs::Sensor newmsg;
	newmsg.name = rostopic;
	newmsg.header.stamp = ros::Time::now();
	newmsg.header.frame_id = "1";
	newmsg.values.push_back(_msg->field_tesla().x());
	newmsg.values.push_back(_msg->field_tesla().y());
	newmsg.values.push_back(_msg->field_tesla().z());
	publisher_.publish(newmsg);
	
}
