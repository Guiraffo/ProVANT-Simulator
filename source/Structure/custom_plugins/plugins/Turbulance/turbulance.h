/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the turbulance class.
 *
 * @author Jonatan Campos
 */

#ifndef TURBULENCE_H
#define TURBULENCE_H

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <eigen3/Eigen/Eigen>

namespace gazebo
{
class turbulance : public ModelPlugin
{
public:
  turbulance();
  virtual ~turbulance() = default;
  // initial setup
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  
  std::string TurbTag;
protected:
	// for each step time
  virtual void Update();

private:
  std::string NameOfTopic_;               // nme of node
  physics::WorldPtr world;                // pointer to the world
  //! Connection to the WorldUpdateBegin event used to call the Update method at every step time.
  event::ConnectionPtr updateConnection;
  ros::NodeHandle node_handle_;           // ROS's node handle
  boost::mutex lock;                      // mutex
  ros::Publisher publisher_;              // ROS publisher
  double delT;
  // Turbulance Model Matrices - Von Karman
  Eigen::VectorXd v;
  Eigen::MatrixXd Ad;
  Eigen::MatrixXd Bd;
  Eigen::MatrixXd Cd;
  Eigen::VectorXd dp;
  Eigen::VectorXd EnvWind;
  //! Message used to identify the log messages of this plugin in the output screen.
  std::string _logMsg;
  //! Name of this plugin child logger.
  const std::string PLUGIN_ID = "turbulence_plugin";
};
}  // namespace gazebo

#endif // TURBULENCE_H
