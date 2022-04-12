/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the turbulance class.
 *
 * @author Jonatan Campos
 */

#include <turbulance.h>

#include <random>

#include "simulator_msgs/Sensor.h"
#include <provant_simulator_xml_reader/config_reader.h>

namespace gazebo
{
turbulance::turbulance() : v(3), Ad(8, 8), Bd(8, 3), Cd(3, 8)
{
  // Defines the Ad,Bd and Cd matrices for Von Karman turbulance ss model
  Ad << -0.090289, -0.00087965, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.084848, -0.00085059, -1.8769e-06, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.64178, -0.048665, -0.00081222, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
  Bd << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
  Cd << 2.2658, 0.11982, 0, 0, 0, 0, 0, 0, 0, 0, 1.9881, 0.10627, 0.00025566, 0, 0, 0, 0, 0, 0, 0, 0, 0.16713, 0.067574,
      0.0012296;
  delT = 0.012;
}

void turbulance::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  _logMsg = "[TurbulencePlugin";
  if (_sdf->HasAttribute("name"))
  {
    _logMsg += ": ";
    _logMsg += _sdf->GetAttribute("name")->GetAsString();
  }
  _logMsg += "] ";

  // Check if ROS is initialized, if not, report the error and cancel the plugin loading process
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                 "plugin. Load the Gazebo system plugin "
                                                 "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package and "
                                                 "try again. Note: This plugin will be automatically loaded using "
                                                 "the \"roslaunch gazebo_ros empty_world\" launch command.");
    return;
  }

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting plugin initialization process.");

  // Get the path of the config.xml file from the ROS parameter server
  std::string configFilePath;
  if (node_handle_.getParam("/provant_simulator/config_file_path", configFilePath) && !configFilePath.empty())
  {
    ConfigReader conf;
    if (conf.open(configFilePath))
    {
      bool found = false;
      std::string turbulenceModel = conf.getTurbulenceModel(&found);
      if (found)
      {
        TurbTag = turbulenceModel.c_str();
      }
      else
      {
        ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error on the turbulence plugin loading process. It was not "
                                                     "possible to read the turbulence model name from the config.xml "
                                                     "file with path: \""
                                                  << configFilePath
                                                  << "\". Verify if the tag with name \"Turbulance\" tag is "
                                                     "defined in the provided config.xml file and try again.");
        return;
      }
    }
    else
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error on the turbulence plugin loading process. The config.xml "
                                                   "file with path \""
                                                << configFilePath
                                                << "\" could not be opened. Please make sure that the path of the "
                                                   "/provant_simulator/config_file_path path is correct, and the "
                                                   "file is readable "
                                                   "for the current user and try again.");
      return;
    }
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error on the turbulence plugin loading process. The "
                                                 "/provant_simulator/config_file_path parameter is not defined. "
                                                 "Please set this parameter with the path to the config.xml file "
                                                 "used to configure the simulation parameters and try again.");
    return;
  }

  world = _model->GetWorld();

  // Get name of topic to publish data
  if (_sdf->HasElement("NameOfTopic"))
  {
    NameOfTopic_ = _sdf->Get<std::string>("NameOfTopic");
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error in the turbulence plugin loading process. The SDF "
                                                 "configuration for the plugin does not contain the required "
                                                 "\"NameOfTopic\" tag. This tag is used to defined the name of the "
                                                 "topic used to publish data collected by this plugin. Please define "
                                                 "this tag and try again.");
    return;
  }

  publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfTopic_, 1);

  updateConnection = event::Events::ConnectWorldUpdateEnd(std::bind(&gazebo::turbulance::Update, this));

  // Inform user that the plugin was successfully loaded
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Turbulence plugin loaded successfully");
}

void turbulance::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);  // mutex

  if (TurbTag == "Custom_Model")
  {
    // User Implementation
  }
  //-------------Implements Von Karman Turbulance Model-------------
  else if (TurbTag == "Von_Karman")
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Von Karman model selected.");
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // Defines a normal distribution
    std::normal_distribution<double> distribution_x(0.0, 0.1);
    std::normal_distribution<double> distribution_y(0.0, 0.1);
    std::normal_distribution<double> distribution_z(0.0, 0.1);

    v << distribution_x(generator), distribution_y(generator), distribution_z(generator);
    static Eigen::VectorXd dp_ant;
    dp_ant = Bd * v;

    static Eigen::VectorXd d(8);
    d << 0, 0, 0, 0, 0, 0, 0, 0;
    static Eigen::VectorXd dpint(8);
    dpint << 0, 0, 0, 0, 0, 0, 0, 0;
    dp = Ad * d + Bd * v;

    dpint = dpint + (delT / 2) * (dp + dp_ant);
    dp_ant = dp;
    d = dpint;

    EnvWind = Cd * dpint;
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "EnvWind value: " << EnvWind);

    simulator_msgs::Sensor newmsg;
    newmsg.name = NameOfTopic_;
    newmsg.header.stamp = ros::Time::now();  // time stamp
    newmsg.header.frame_id = "1";
    newmsg.values.push_back(EnvWind(0));  // x
    newmsg.values.push_back(EnvWind(1));  // y
    newmsg.values.push_back(EnvWind(2));  // z

    publisher_.publish(newmsg);
  }
  else if (TurbTag == "Turbulance Model 2")
  {
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Unrecognized turbulence model: " << TurbTag);
  }
}

GZ_REGISTER_MODEL_PLUGIN(turbulance)
}  // namespace gazebo
