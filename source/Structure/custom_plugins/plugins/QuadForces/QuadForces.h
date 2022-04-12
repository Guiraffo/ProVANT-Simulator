/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the QuadForces class.
 *
 * @author Jonatan Mota Campos
 */

#ifndef QUAD_FORCES_H
#define QUAD_FORCES_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <limits>

#include <eigen3/Eigen/Eigen>

namespace gazebo
{
/**
 * @brief The QuadForces class is a Gazebo Model Plugin that can apply the forces generated by the propellers
 * to a quadrotor UAV.
 *
 * This plugin receives the forces applied to the four brushless motors of a quadrotor Unmanned Aerial Vehicle (UAV),
 * specified in Newtons, and calculates the forces and torques that the forces applied by the propeller will generate
 * in the UAV center of rotation.
 *
 * The calculated forces are then applied at every simulation step, using the latest values received in the topics of
 * each propeller.
 *
 * The torque calculation assumes that the torque generated by each propeller is proportional to the force generated by
 * the propeller by a drag constant, using the following equation:
 * @f[
 * 	\tau_i = k_{\tau}\f_i,
 * @f]
 * where \f$ k_{\tau} \f$ is the drag constant of the propellers, \f$ \tau_i \f$ is the torque generated by the i-th
 * propeller, and \f$ f_i \f$ is the force generated by the i-th propeller.
 */
class QuadForces : public ModelPlugin
{
public:
  /**
   * @brief Construct a new Quad Forces object.
   * Initializes the size of the matrices used in the calculation.
   */
  QuadForces();
  /**
   * @brief Destroy the Quad Forces object.
   * Cancel subscription to the ROS topics, and also the subscription to the Gazebo WorldUpdateBegin event.
   */
  virtual ~QuadForces() = default;
  /**
   * @brief Method called when the plugin is Loaded by Gazebo.
   *
   * Reads the tags configured in the SDF model, subscribe to the ROS topics used to receive the force values,
   * and connects to the Gazebo WorldUpdateBegin event.
   *
   * @param _model Pointer to the Model that contains the plugin.
   * @param _sdf Pointer to the SDF element that instantiates the plugin.
   */
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  /**
   * @brief Method called when a new force value is published at the ROS topics used to set the value of the first
   * propeller.
   *
   * @param msg Message containg the force value for the first propeller.
   */
  void CallbackF1(const std_msgs::Float64& msg);
  /**
   * @brief Method called when a new force value is published at the ROS topics used to set the value of the second
   * propeller.
   *
   * @param msg Message containg the force value for the second propeller.
   */
  void CallbackF2(const std_msgs::Float64& msg);
  /**
   * @brief Method called when a new force value is published at the ROS topics used to set the value of the third
   * propeller.
   *
   * @param msg Message containg the force value for the third propeller.
   */
  void CallbackF3(const std_msgs::Float64& msg);
  /**
   * @brief Method called when a new force value is published at the ROS topics used to set the value of the fourth
   * propeller.
   *
   * @param msg Message containg the force value for the fourth propeller.
   */
  void CallbackF4(const std_msgs::Float64& msg);

protected:
  /**
   * @brief Method called by Gazebo at every simulation step.
   *
   * Calculates and applies the forces and torques applied to the quadrotor UAV center of rotation using the latest
   * values of the forces received by the plugin.
   *
   */
  void Update();
  /**
   * @brief Applies saturation to the force received acoording to the values configured in the class.
   *
   * If saturation is disabled, this method returns the same value of force as it receives.
   * If saturation is enabled, and the force is greater than the maximum allowed force value, the maximum value is
   * returned, if the force is smaller than the minimum allowed value, the minimum is returned, and otherwise, the
   * unsaturated force value is returned.
   *
   * @param force Unsaturated force value (in Newtons).
   * @return double Saturated force value (in Newtons).
   */
  double SaturateForce(double force) const;
	/**
	 * @brief Method called during plugin loading to setup the saturation of the propellers.
	 *
	 * Reads the values of the tags saturate, satMin, and satMax, and setups the appropriate values in the member
	 * variables.
	 *
	 * @param sdf Pointer to the SDF element that instantiates the plugin and contain its properties.
	 */
	void SetupSaturation(sdf::ElementPtr sdf);

private:
  //! ROS node handle to manage the topics subscribed by this plugin
  ros::NodeHandle node_handle_;
  //! Pointer to the link where the forces will be applied.
  physics::LinkPtr link;
  //! Pointer to the simulation world
  physics::WorldPtr _world;

  //! Name of the topic used to receive the forces of the first brushless
  std::string topic_F1;
  //! Name of the topic used to receive the forces of the second brushless
  std::string topic_F2;
  //! Name of the topic used to receive the forces of the third brushless
  std::string topic_F3;
  //! Name of the topic used to receive the forces of the fourth brushless
  std::string topic_F4;

  //! ROS subscriber to receive the forces applied to the first brushless
  ros::Subscriber motor_subscriberF1_;
  //! ROS subscriber to receive the forces applied to the second brushless
  ros::Subscriber motor_subscriberF2_;
  //! ROS subscriber to receive the forces applied to the third brushless
  ros::Subscriber motor_subscriberF3_;
  //! ROS subscriber to receive the forces applied to the fourth brushless
  ros::Subscriber motor_subscriberF4_;

  //! Name of the link where the forces will be applied.
  std::string NameOfLink_;

  //! Connection pointer to the Gazebo WorldUpdateBegin event used to run the Update method every simulation step.
  event::ConnectionPtr updateConnection;

  //! Tilt angle of the propellers towards the UAV center of rotation
  double alpha;
  //! Arm legnth of the UAV, ie. the distance from the propellers to the center of rotation.
  double length;
  //! Vector of the forces applied to the UAV center of rotation.
  Eigen::VectorXd ForceBody;
  //! Vector of the troques applied to the UAV center of rotation.
  Eigen::VectorXd TorqueBody;
  //! Forces of the propellers.
  double F1, F2, F3, F4;
  //! Drag constant of the propellers, used to calculate the torque generated by each propeller.
  double DragConst;
  //! Indicates if saturation mus tbe applied to the forces or not.
  bool _saturate = false;
  //! Minimum value of the forces that can be generated by the propellers.
  double _satMin = -std::numeric_limits<double>::infinity();
  //! Maximum value of the forces that can be generated by the propellers.
  double _satMax = std::numeric_limits<double>::infinity();
  //! String used to identify the log messages sent by this plugin.
  std::string _logMsg;
  //! String used to name the custom logger for this plugin
  const std::string PLUGIN_ID = "quad_forces_plugin";
};
}  // namespace gazebo

#endif  // QUAD_FORCES_H
