/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file AllData2.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * reads the state-space data of a UAV with an attached load.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_ALLDATA2_H
#define PROVANT_ALLDATA2_H

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <boost/thread.hpp>

namespace gazebo
{
class AllData2 : public ModelPlugin
{
public:
  AllData2() = default;
  virtual ~AllData2() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  virtual void Update();

private:
  std::string NameOfJointLoad_X_;         // name of joint gammax
  std::string NameOfJointLoad_Y_;         // name of joint gammay
  std::string NameOfJointR_;              // name of right joint
  std::string NameOfJointL_;              // name of left joint
  std::string NameOfNode_;                // name of ROS node
  std::string link_name_;                 // name of main body
  physics::LinkPtr link;                  // main body's link
  physics::WorldPtr world;                // pointer to world
  physics::JointPtr juntaR;               // pointer to right joint
  physics::JointPtr juntaL;               // pointer to left joint
  physics::JointPtr juntaLoadX;           // pointer to joint gammax
  physics::JointPtr juntaLoadY;           // pointer to right gammay
  event::ConnectionPtr updateConnection;  // update connection
  ros::NodeHandle node_handle_;           // ROS's node handle
  boost::mutex lock;                      // mutex
  ros::Publisher publisher_;              // publisher
};
}  // namespace gazebo

#endif  // PROVANT_ALLDATA2_H
