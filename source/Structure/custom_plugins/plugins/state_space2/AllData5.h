/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file AllData5.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * publishes the state vector of the UAV5 on a ROS topic as a sensor message.
 *
 * @author Jonatan Campos
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_ALL_DATA5_H
#define PROVANT_ALL_DATA5_H

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <random>

namespace gazebo
{
class AllData5 : public ModelPlugin
{
  std::fstream out;
  time_t timev;
  std::default_random_engine generator;
  std::normal_distribution<double> distributionX;
  std::normal_distribution<double> distributionY;
  std::normal_distribution<double> distributionZ;

public:
  AllData5();
  virtual ~AllData5() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  virtual void Update();

private:
  // variáveis para armazenar os nomes:
  std::string NameOfJointR_;
  std::string NameOfJointL_;
  std::string NameOfNode_;
  std::string link_name_;

  // variáveis físicas
  physics::LinkPtr link;
  physics::WorldPtr world;
  physics::JointPtr juntaR;
  physics::JointPtr juntaL;

  event::ConnectionPtr updateConnection;
  ros::NodeHandle node_handle_;
  boost::mutex lock;
  ros::Publisher publisher_;
};
}  // namespace gazebo

#endif  // PROVANT_ALL_DATA5_H
