/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file imu.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * reads the IMU Data for a particular model.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <simulator_msgs/Sensor.h>

namespace gazebo
{
/**
 * @todo Check if this plugin is necessary. If it is the number of commented out lines indicates a redesign is probably
 * necessary.
 */
class imu : public ModelPlugin
{
public:
  imu() = default;
  virtual ~imu() = default;
  // initial setup
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  // callback for each time step
  virtual void Update();

private:
  std::string link_name_;                 // link name
  physics::LinkPtr link;                  // link
  physics::WorldPtr world;                // world pointer
  event::ConnectionPtr updateConnection;  // pointer to update connection
  ros::NodeHandle n;                      // ROS node handle
  ros::Publisher imu_pub;                 // ROS publisher
  std::string Topic_;                     // ROS topic
};
}  // namespace gazebo
