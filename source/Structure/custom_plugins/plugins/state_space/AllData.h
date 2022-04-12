/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file AllData.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * reads the following state-space data from a UAV, x, y, z, roll, pitch, yaw,
 * angle of the left rotor, angle of the right tor, and their derivatives.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_ALLDATA_H
#define PROVANT_ALLDATA_H

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include <boost/thread.hpp>

namespace gazebo
{
class AllData : public ModelPlugin
{
  // constructor
public:
  AllData();
  virtual ~AllData() = default;
  // initial setup
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  virtual void Update();

private:
  std::string NameOfJointR_;              // name of right joint
  std::string NameOfJointL_;              // name od left joint
  std::string NameOfNode_;                // nme of node
  std::string link_name_;                 // name of link
  physics::LinkPtr link;                  // pointer to the link
  physics::WorldPtr world;                // pointer to the world
  physics::JointPtr juntaR;               // poiter to the right joint
  physics::JointPtr juntaL;               // pointer to the left joint
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

#endif  // PROVANT_ALLDATA_H
