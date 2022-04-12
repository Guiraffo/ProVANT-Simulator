/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file QuadData.h
 * @brief This file contains the declaration of the QuadData class.
 *
 * The QuadData class is a Gazebo (version 9) ModelPlugin that implements a
 * sensor that reads the states of a quadrotor UAV.
 * The state vector is then published in a ROS topic, in the following order:
 * x, y, z, roll, pitch, yaw, dx, dy, dz, dot_roll, dot_yaw, dot_pitch, dot_yaw,
 * u, v, w, p, q, r, qx, qy, qz, qw
 *
 * Where:
 * 	x, y, z are the linear position of the quadrotor center of rotation, in
 * 		meters.
 * 	roll, pitch, yaw are the Euler angles that denotes the attitude of the
 * 		quadorotor UAV center of rotation in relation with the inertial frame.
 * 	dx, dy, dz are the linear velocities (first derivatives) of the quadrotor
 * 		UAV center of rotation in relation with the inertial frame.
 * 	dot_roll, dot_yaw, dot_pitch are the first temporal derivatives of the
 * 		Euler Angles
 * 	u, v, w are the angular velocities of the quadrotor UAV center of rotation,
 * 		in relation with the inertial frame, expressed in the inertial frame.
 * 	p, q, r are the angular velocities of the quadrotor UAV center of rotation,
 * 		in relation with the inertial frame, expressed in the body fixed frame.
 * 	qx, qy, qz, qw are the orientation quaternion of the quadortor UAV center of
 * 		rotation, in relation with the inertial frame, in the following order
 * 		qx * i + qy * j + qz * k + qw, where i, j, k, are the standard quaternion
 * 		imaginary units (i^2 = j^2 = k^2 = -1).
 *
 * @author Jonatan Mota Campos
 */

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include <boost/thread.hpp>

#include <string>

namespace gazebo
{
/**
 * @brief The QuadData class is a state vector sensor for a quadrotor UAV.
 * See the file documentation for more details.
 */
class QuadData : public ModelPlugin
{
public:
  /**
   * @brief Construct a new Quad Data object.
   * Initializes the sizes of the matrices used in the plugin.
   */
  QuadData();
  /**
   * @brief Destroy the Quad Data object.
   */
  virtual ~QuadData() = default;
  /**
   * @brief Method called when the plugin is loaded by the Gazebo simulator.
   *
   * @param _model Pointer to the model containing this plugin.
   * @param _sdf Pointer to the SDF (Simulation Description File) that defines
   * the inclusion of this plugin, and its options. Note: A SDF is a specialized
   * XMl file and has the same syntax.
   */
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief The Update method is called every simulation step.
   *
   * This method must obtain the data contained in the quadrotor UAV state
   * vector and publish a message in a ROS topic containing this data.
   */
  virtual void Update();

private:
  //! Stores the name of the node used to publish the state vector.
  std::string NameOfNode_;
  //! Stores the name of the link containing the quadrotor UAV center of rotation.
  std::string link_name_;
  //! Stores a pointer to the link containing the quadrotor UAV center of rotation.
  physics::LinkPtr link;
  //! Stores a pointer to the simulation world.
  physics::WorldPtr world;
  //! Stores the connection to the update timer.
  event::ConnectionPtr updateConnection;
  //! ROS Node Handle used to manage the topics and services provided and consumed by this node.
  ros::NodeHandle node_handle_;
  //! Mutex used to ensure only one thread can call the Update method
  boost::mutex lock;
  //! ROS publisher used to publish the state vector.
  ros::Publisher publisher_;

  //! Rotation Matrix from the body fixed frame to the inertial frame.
  Eigen::MatrixXd RIB;
  //! Euler Matrix that maps the angular velocities of the quadrotor UAV to the first temporal derivatives of the Euler
  //! Angles.
  Eigen::MatrixXd W_n;
  //! Vector of the angular velocities of the quadrotor UAV center of rotation, in relation with the inertial frame, and
  //! expressed in the World Frame.
  Eigen::MatrixXd WIIB;
  //! Vector of the first temporal derivatives of the Euler Angles.
  Eigen::MatrixXd PhipThetapPsip;
  //! Vector of the linear velocities of the quadrotor UAV center of rotation, in relation with the inertial frame.
  Eigen::MatrixXd XpYpZp;
  //! Vector of the linear velocities of the quadroror UAV center of rotation, in relation with the inertial frame, and
  //! expressed in the body fixed frame.
  Eigen::MatrixXd UVW;
  //! Vector of the angular velocities of the quadrotor UAV center of rotation, in relation with the inertial frame, and
  //! expressed in the body fixed frame.
  Eigen::MatrixXd PQR;
  //! Roll Euler Angle
  double Phi;
  //! Pitch Euler Angle
  double Theta;
  //! Yaw Euler Angle
  double Psi;
  //! Message used to identify this plugin in the log messages
  std::string _logMsg;
  //! Name of the plugin child logger
  const std::string PLUGIN_ID = "quad_data_plugin";
};
}  // namespace gazebo
