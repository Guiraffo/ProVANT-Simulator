/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file DataSaveTiltRotor.h
 * @brief This file contains the implementation of the DataSaveTiltRotor class.
 *
 * The DataSaveTiltRotor class implements a Gazebo Model plugin that receives,
 * saturates and saves to a data file the control inputs applied to a tilt rotor
 * UAV.
 *
 * @author Jonatan Mota Campos
 */

#ifndef PROVANT_DATA_SAVE_TILT_ROTOR_H
#define PROVANT_DATA_SAVE_TILT_ROTOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Eigen>

#include <MatlabData.h>
#include <XMLRead.h>

namespace gazebo
{
class DataSaveTiltRotor : public ModelPlugin
{
public:
  DataSaveTiltRotor();                                                 // constructor
  virtual ~DataSaveTiltRotor();                                        // destructor
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;  // initial setup
  void CallbackFr(std_msgs::Float64);    // callback to saturate and save forces at right brushless
  void CallbackFl(std_msgs::Float64);    // callback to saturate and save forces at left brushless
  void CallbackRotr(std_msgs::Float64);  // callback to saturate and save deflection at right rotor
  void CallbackRotl(std_msgs::Float64);  // callback to saturate and save deflection at left rotor
  void CallbackDAr(std_msgs::Float64);   // callback to saturate and save deflection at right aileron
  void CallbackDAl(std_msgs::Float64);   // callback to saturate and save deflection at left aileron
  void CallbackDRr(std_msgs::Float64);   // callback to saturate and save deflection at right ruddervator
  void CallbackDRl(std_msgs::Float64);   // callback to saturate and save deflection at left ruddervator
  void Update();

private:
  ros::NodeHandle node_handle_;  // node handle of ROS
  physics::WorldPtr world;       // pointer to the simulation world

  std::string topic_Fr;    // name of topic of right brushless
  std::string topic_Fl;    // name of topic of left brushless
  std::string topic_Rotr;  // name of topic of right rotor
  std::string topic_Rotl;  // name of topic of left rotor
  std::string topic_DAr;   // name of topic of right aileron
  std::string topic_DAl;   // name of topic of left aileron
  std::string topic_DRr;   // name of topic of right ruddervator
  std::string topic_DRl;   // name of topic of left ruddervator

  // ROS subscribers
  ros::Subscriber subscriberFr_;
  ros::Subscriber subscriberFl_;
  ros::Subscriber subscriberRotr_;
  ros::Subscriber subscriberRotl_;
  ros::Subscriber subscriberDAr_;
  ros::Subscriber subscriberDAl_;
  ros::Subscriber subscriberDRr_;
  ros::Subscriber subscriberDRl_;

  double Fr, Fl, DAr, DAl, DRr, DRl, Rotr, Rotl;                                  // Values to be satureted and saved
  double Fr_sat, Fl_sat, DAr_sat, DAl_sat, DRr_sat, DRl_sat, Rotr_sat, Rotl_sat;  // Saturation values

  // Files for saving data
  MatlabData FrFile;
  MatlabData FlFile;
  MatlabData RotrFile;
  MatlabData RotlFile;
  MatlabData DArFile;
  MatlabData DAlFile;
  MatlabData DRrFile;
  MatlabData DRlFile;
};
}  // namespace gazebo

#endif  // PROVANT_DATA_SAVE_TILT_ROTOR_H
