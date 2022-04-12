/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file DataSave.h
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_DATA_SAVE_H
#define PROVANT_DATA_SAVE_H

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Eigen>

#include "MatlabData.h"

namespace gazebo
{
class DataSave : public ModelPlugin
{
public:
  DataSave();
  ~DataSave();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate();

private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
  physics::WorldPtr world;
  physics::LinkPtr link;      // pointer to the link
  std::string NameOfJointR_;  // name of right joint
  std::string NameOfJointL_;  // name od left joint
  std::string NameOfNode_;    // nme of node
  std::string link_name_;     // name of link
  physics::JointPtr juntaR;   // poiter to the right joint
  physics::JointPtr juntaL;   // pointer to the left joint
  Eigen::MatrixXd RIB;
  Eigen::MatrixXd W_n;
  Eigen::MatrixXd WIIB;
  Eigen::MatrixXd PhipThetapPsip;
  Eigen::MatrixXd XpYpZp;
  Eigen::MatrixXd UVW;
  Eigen::MatrixXd PQR;
  MatlabData GeneralizedCoordinatesNVelocities;
  MatlabData GeneralizedVelocitiesExpressedOnbodyFrame;
  Eigen::VectorXd q;
  Eigen::VectorXd qp;      // x y z phi theta psi ar al
  Eigen::VectorXd qpBody;  // x y z phi theta psi ar al
  Eigen::VectorXd u;
  double Phi;
  double Theta;
  double Psi;
};
}  // namespace gazebo

#endif  // PROVANT_DATA_SAVE_H
