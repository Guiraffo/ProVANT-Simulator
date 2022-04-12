/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file UniversalLinkSensor.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * reads all of the data for a specific link.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_UNIVERSAL_LINK_SENSOR_H
#define PROVANT_UNIVERSAL_LINK_SENSOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <boost/thread.hpp>

namespace gazebo
{
class UniversalLinkSensor : public ModelPlugin
{
public:
  UniversalLinkSensor() = default;
  virtual ~UniversalLinkSensor() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  // for each step time
  virtual void Update();

private:
  std::string NameOfNode_;                // name of node
  std::string link_name_;                 // name of link
  physics::LinkPtr link;                  // pointer to the link
  physics::WorldPtr world;                // pointer to the world
  event::ConnectionPtr updateConnection;  // update connection
  ros::NodeHandle node_handle_;           // ROS's node handle
  boost::mutex lock;                      // mutex
  ros::Publisher publisher_;              // publisher
};
}  // namespace gazebo

#endif  // PROVANT_UNIVERSAL_LINK_SENSOR_H
