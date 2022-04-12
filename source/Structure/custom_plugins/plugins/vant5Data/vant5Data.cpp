/*
* File: vant5Data.cpp
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 13/10/2020
* Description:  This library is responsable to implement a sensor that return all data of the folowing state space:

- x,y,z,roll,pitch,yaw,aR2,aL3,aR4,aL5,dx,dy,dz,droll,dpitch,dyaw,daR2,daL3,daR4,daL5

*/

#include <vant5Data.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>

#include <eigen3/Eigen/Eigen>

namespace gazebo
{
// constructor
vant5Data::vant5Data() : PhipThetapPsip(3, 1), RIB(3, 3), W_n(3, 3), WIIB(3, 1)
{
}

// initial setup
void vant5Data::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "Estados Initialized" << std::endl;
  if (!ros::isInitialized())
  {
    ROS_INFO("Nao inicializado!");
    return;
  }

  NameOfTopic_ = XMLRead::ReadXMLString("NameOfTopic", _sdf);       // Get name of topic to publish data
  NameOfJointR2_ = XMLRead::ReadXMLString("NameOfJointR_2", _sdf);  // name of the right frontal joint
  NameOfJointL3_ = XMLRead::ReadXMLString("NameOfJointL_3", _sdf);  // name of the left frontal joint
  NameOfJointR4_ = XMLRead::ReadXMLString("NameOfJointR_4", _sdf);  // name of the right back joint
  NameOfJointL5_ = XMLRead::ReadXMLString("NameOfJointL_5", _sdf);  // name of the left back joint

  world = _model->GetWorld();                  // pointer to the world
  juntaR2 = _model->GetJoint(NameOfJointR2_);  // pointer to the right frontal joint
  juntaL3 = _model->GetJoint(NameOfJointL3_);  // pointer to the left frontal joint
  juntaR4 = _model->GetJoint(NameOfJointR4_);  // pointer to the right back joint
  juntaL5 = _model->GetJoint(NameOfJointL5_);  // pointer to the left back joint

  link_name_ = XMLRead::ReadXMLString("bodyName", _sdf);  // name of the main body
  link = _model->GetLink(link_name_);                     // pointer to the main body

  // Connect to the WorldUpdateEnd event to call the Update function at every new step time
  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });

  // ROS publisher
  publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfTopic_, 1);
}

// new step time
void vant5Data::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);  // mutex

  simulator_msgs::Sensor newmsg;
  newmsg.name = NameOfTopic_;
  newmsg.header.stamp = ros::Time::now();  // time stamp
  newmsg.header.frame_id = "1";
  ignition::math::Pose3d pose = link->WorldPose();
  newmsg.values.push_back(pose.Pos().X());          // x
  newmsg.values.push_back(pose.Pos().Y());          // y
  newmsg.values.push_back(pose.Pos().Z());          // z
  newmsg.values.push_back(pose.Rot().Euler().X());  // roll
  newmsg.values.push_back(pose.Rot().Euler().Y());  // pitch
  newmsg.values.push_back(pose.Rot().Euler().Z());  // yaw
  newmsg.values.push_back(juntaR2->Position(0));    // aR2
  newmsg.values.push_back(juntaL3->Position(0));    // aL3
  newmsg.values.push_back(juntaR4->Position(0));    // aR4
  newmsg.values.push_back(juntaL5->Position(0));    // aL5

  ignition::math::Vector3d linear = link->WorldLinearVel();
  newmsg.values.push_back(linear.X());  // vx
  newmsg.values.push_back(linear.Y());  // vy
  newmsg.values.push_back(linear.Z());  // vz
  ignition::math::Vector3d angular = link->WorldAngularVel();

  // Maps to the body
  Phi = pose.Rot().Euler().X();
  Theta = pose.Rot().Euler().Y();
  Psi = pose.Rot().Euler().Z();

  RIB << (cos(Psi) * cos(Theta)), (cos(Psi) * sin(Phi) * sin(Theta) - cos(Phi) * sin(Psi)),
      (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)), (cos(Theta) * sin(Psi)),
      (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)),
      (cos(Phi) * sin(Psi) * sin(Theta) - cos(Psi) * sin(Phi)), (-sin(Theta)), (cos(Theta) * sin(Phi)),
      (cos(Phi) * cos(Theta));

  W_n << 1.0, 0.0, -sin(Theta), 0.0, cos(Phi), cos(Theta) * sin(Phi), 0.0, -sin(Phi), cos(Phi) * cos(Theta);

  WIIB << angular.X(), angular.Y(), angular.Z();
  PhipThetapPsip = W_n.inverse() * RIB.transpose() * WIIB;

  newmsg.values.push_back(PhipThetapPsip(0));
  newmsg.values.push_back(PhipThetapPsip(1));
  newmsg.values.push_back(PhipThetapPsip(2));

  newmsg.values.push_back(juntaR2->GetVelocity(0));  // daR2
  newmsg.values.push_back(juntaL3->GetVelocity(0));  // daL3
  newmsg.values.push_back(juntaR4->GetVelocity(0));  // daR4
  newmsg.values.push_back(juntaL5->GetVelocity(0));  // daL5

  // publish data
  publisher_.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(vant5Data)
}  // namespace gazebo
