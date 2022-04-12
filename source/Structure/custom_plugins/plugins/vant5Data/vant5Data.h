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

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <boost/thread.hpp>
#include <random>
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"

#include <eigen3/Eigen/Eigen>

namespace gazebo
{
class vant5Data : public ModelPlugin
{
public:
  vant5Data();
  virtual ~vant5Data() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  // for each step time
  virtual void Update();

private:
  std::string NameOfJointR2_;             // name of right frontal joint
  std::string NameOfJointL3_;             // name od left frontal joint
  std::string NameOfJointR4_;             // name of right back joint
  std::string NameOfJointL5_;             // name of left back joint
  std::string NameOfTopic_;               // name of topic
  std::string link_name_;                 // name of link
  physics::LinkPtr link;                  // pointer to the link
  physics::WorldPtr world;                // pointer to the world
  physics::JointPtr juntaR2;              // poiter to the right frontal joint
  physics::JointPtr juntaL3;              // pointer to the left frontal joint
  physics::JointPtr juntaR4;              // poiter to the right back joint
  physics::JointPtr juntaL5;              // pointer to the left back joint
  event::ConnectionPtr updateConnection;  // update connection
  ros::NodeHandle node_handle_;           // ROS's node handle
  boost::mutex lock;                      // mutex
  ros::Publisher publisher_;              // ROS publisher
  Eigen::MatrixXd RIB;
  Eigen::MatrixXd W_n;
  Eigen::MatrixXd WIIB;
  Eigen::MatrixXd PhipThetapPsip;
  double Phi;
  double Theta;
  double Psi;
};
}  // namespace gazebo
