#include "gps.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(gps)

/////////////////////////////////////////////////
gps::gps() 
{

}

/////////////////////////////////////////////////
gps::~gps()
{
	//gazebo::client::shutdown();
}

/////////////////////////////////////////////////
void gps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	// GET XML INFO
	gazebotopic = XMLRead::ReadXMLString("gazebotopic",_sdf);
	rostopic = XMLRead::ReadXMLString("rostopic",_sdf);
	std::string link = XMLRead::ReadXMLString("link",_sdf); 
        this->model = _model;
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->GetName());

	// OTHERS TOOLS (ROS, GAZEBO)
        std::string topic = "/gazebo/default/" + this->model->GetName() + "/" + link + "/" + gazebotopic;
        this->sub = this->node->Subscribe(topic, &gps::OnUpdate, this);
	publisher_ = n.advertise<simulator_msgs::Sensor>(rostopic, 1);
}

/////////////////////////////////////////////////
void gps::OnUpdate(ConstGPSPtr &_msg)
{
	simulator_msgs::Sensor newmsg;
	newmsg.name = rostopic;
	newmsg.header.stamp = ros::Time::now();
	newmsg.header.frame_id = "1";
	newmsg.values.push_back(_msg->latitude_deg());
	newmsg.values.push_back(_msg->longitude_deg());
	newmsg.values.push_back(_msg->altitude());
	newmsg.values.push_back(_msg->velocity_east());
	newmsg.values.push_back(_msg->velocity_north());
	newmsg.values.push_back(_msg->velocity_up());
	publisher_.publish(newmsg);

}
