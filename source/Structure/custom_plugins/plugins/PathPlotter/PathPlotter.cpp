/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the PathPlotter class.
 *
 * @author Jonatan Mota Campos
 */

#include <PathPlotter.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "XMLRead.h"

namespace gazebo
{
// initial setup
void PathPlotter::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != nullptr, "Received a null model pointer");
  GZ_ASSERT(_sdf != nullptr, "Received empty SDF element pointer");

  try
  {
    _logMsg = "[PathPlotterPlugin";
    if (_sdf->HasAttribute("name"))
    {
      _logMsg += ": ";
      _logMsg += _sdf->GetAttribute("name")->GetAsString();
    }
    _logMsg += "] ";

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting plugin loading process.");
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                   "plugin. Load the Gazebo system plugin "
                                                   "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package and "
                                                   "try again. Note: This plugin will be automatically loaded using "
                                                   "the \"roslaunch gazebo_ros empty_world\" launch command.");
      return;
    }

    NameOftopic_path_ = XMLRead::ReadXMLString("NameOfPathTopic", _sdf);    // Get name of topic to publish data
    NameOftopic_mark_ = XMLRead::ReadXMLString("NameOfMarkerTopic", _sdf);  // Get name of topic to publish data
    NameOftopic_ref_ = XMLRead::ReadXMLString("NameOfPathRefTopic", _sdf);  // Get name of topic to publish data
    tag_ = XMLRead::ReadXMLString("uav", _sdf);                             // tag to identify which uav we're using

    world = _model->GetWorld();  // pointer to the world
    GZ_ASSERT(world != nullptr, "The model is a null world");

    link_name_ = XMLRead::ReadXMLString("bodyName", _sdf);  // name of the main body
    link = _model->GetLink(link_name_);                     // pointer to the main body

    // ROS publisher
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the /" << NameOftopic_path_ << " ROS topic.");
    publisher_path_ = node_handle_.advertise<nav_msgs::Path>(
        NameOftopic_path_, 1, std::bind(&gazebo::PathPlotter::OnPublisherPathConnect, this),
        std::bind(&gazebo::PathPlotter::OnPublisherPathDisconnect, this));
    _pathPublisherSubscribers = static_cast<int>(publisher_path_.getNumSubscribers());

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the /" << NameOftopic_mark_ << " ROS topic.");
    publisher_marker_ = node_handle_.advertise<visualization_msgs::Marker>(
        NameOftopic_mark_, 1, std::bind(&gazebo::PathPlotter::OnPublisherMarkerConnect, this),
        std::bind(&gazebo::PathPlotter::OnPublisherMarkerDisconnect, this));
    _markerPublisherSubscribers = static_cast<int>(publisher_marker_.getNumSubscribers());

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the /" << NameOftopic_ref_ << " ROS topic.");
    publisher_path_ref_ = node_handle_.advertise<nav_msgs::Path>(
        NameOftopic_ref_, 1, std::bind(&gazebo::PathPlotter::OnPublisherPathRefConnect, this),
        std::bind(&gazebo::PathPlotter::OnPublisherPathRefDisconnect, this));
    _refPublisherSubscribers = static_cast<int>(publisher_path_ref_.getNumSubscribers());

    // Connect to the WorldUpdateEnd event
    updateConnection = event::Events::ConnectWorldUpdateEnd(std::bind(&gazebo::PathPlotter::Update, this));

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded successfully.");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An unexpected expcetion ocurred at the Load method with message: \""
                                              << e.what() << "\".");
  }
}

/**********************************************************************************************************************/

// new step time
void PathPlotter::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);  // mutex
  ignition::math::Pose3d pose = link->WorldPose();
  ros::Time time = ros::Time::now();
  double tempo = time.toSec();

  // broadcast for the reference trajectory
  tf::TransformBroadcaster broadcaster_ref;
  if (tag_ == "vant_4_aerod")
  {
    broadcaster_ref.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(4 * sin(tempo / 2), 4 * cos(tempo / 2), tempo / 10)),
        ros::Time::now(), "inertial_frame", "base_link"));
  }
  else if (tag_ == "quadcopter")
  {
    broadcaster_ref.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(6 * sin(tempo / 2), 6 * cos(tempo / 2), tempo / 3)),
        ros::Time::now(), "inertial_frame", "base_link"));
  }

  // If any subscriber is connected to the publisher_path_ref_ topic, public the necessary messages on it
  if (_refPublisherSubscribers > 0)
  {
    nav_msgs::Path path_ref;
    if (tag_ == "quadcopter")
    {
      geometry_msgs::PoseStamped pose_path_ref;
      pose_path_ref.header.stamp = ros::Time::now();
      pose_path_ref.header.frame_id = "inertial_frame";
      pose_path_ref.pose.position.x = sin(tempo / 2);
      pose_path_ref.pose.position.y = cos(tempo / 2);
      pose_path_ref.pose.position.z = tempo / 10;
      pose_path_ref.pose.orientation.x = 0;
      pose_path_ref.pose.orientation.y = 0;
      pose_path_ref.pose.orientation.z = 0;
      pose_path_ref.pose.orientation.w = 1;
      path_ref.header.stamp = ros::Time::now();
      path_ref.header.frame_id = "inertial_frame";
      path_ref.poses.push_back(pose_path_ref);
      // Publishes the reference trajectory
      publisher_path_ref_.publish(path_ref);
    }
    else if (tag_ == "vant_4_aerod")
    {
      geometry_msgs::PoseStamped pose_path_ref;
      pose_path_ref.header.stamp = ros::Time::now();
      pose_path_ref.header.frame_id = "inertial_frame";
      pose_path_ref.pose.position.x = 6 * sin(tempo / 2);
      pose_path_ref.pose.position.y = 6 * cos(tempo / 2);
      pose_path_ref.pose.position.z = tempo / 3;
      pose_path_ref.pose.orientation.x = 0;
      pose_path_ref.pose.orientation.y = 0;
      pose_path_ref.pose.orientation.z = 0;
      pose_path_ref.pose.orientation.w = 1;
      path_ref.header.stamp = ros::Time::now();
      path_ref.header.frame_id = "inertial_frame";
      path_ref.poses.push_back(pose_path_ref);
      // Publishes the reference trajectory
      publisher_path_ref_.publish(path_ref);
    }
  }

  // broadcaster for the real trajectory
  tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(pose.Rot().Euler().X(), pose.Rot().Euler().Y(), pose.Rot().Euler().Z(), 1),
                    tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z())),
      ros::Time::now(), "inertial_frame", "base_link"));

  nav_msgs::Path path;

  // Publishes the real trajectory
  if (_pathPublisherSubscribers > 0)
  {
    geometry_msgs::PoseStamped pose_path;
    pose_path.header.stamp = ros::Time::now();
    pose_path.header.frame_id = "inertial_frame";
    pose_path.pose.position.x = pose.Pos().X();
    pose_path.pose.position.y = pose.Pos().Y();
    pose_path.pose.position.z = pose.Pos().Z();
    pose_path.pose.orientation.x = pose.Rot().Euler().X();
    pose_path.pose.orientation.y = pose.Rot().Euler().Y();
    pose_path.pose.orientation.z = pose.Rot().Euler().Z();
    pose_path.pose.orientation.w = 1;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "inertial_frame";
    path.poses.push_back(pose_path);

    publisher_path_.publish(path);
  }

  // Marker for visualization
  if (_refPublisherSubscribers > 0)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "base_link";
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = marker.MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1;
    if (tag_ == "vant_4_aerod")
    {
      marker.mesh_resource = "package://Database/models/vant_4_aerod/meshes/mainbodyrviz.dae";
    }
    else if (tag_ == "quadcopter")
    {
      marker.mesh_resource = "package://Database/models/quadcopter/meshes/main_body.dae";
    }
    publisher_marker_.publish(marker);
  }
}

void PathPlotter::OnPublisherPathConnect()
{
  boost::mutex::scoped_lock lockguard(_pathPublisherLock);
  if (_pathPublisherSubscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "A new subscriber connected to the /" << NameOftopic_path_
                                             << " ROS topic. The path plotter plugin will not publish messages in this "
                                                "topic.");
  }
  _pathPublisherSubscribers++;
}

void PathPlotter::OnPublisherPathDisconnect()
{
  boost::mutex::scoped_lock lockguard(_pathPublisherLock);
  _pathPublisherSubscribers--;
  if (_pathPublisherSubscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The last subscriber disconnected from the /" << NameOftopic_path_
                                             << " ROS topic. The path plotter plugin will no longer publish messages "
                                                "in this "
                                                "topic.");
  }
  else if (_pathPublisherSubscribers < 0)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error on the path publisher topic subscriber update. A negative "
                                                 "subscriber count was detecetd.");
  }
}

void PathPlotter::OnPublisherPathRefConnect()
{
  boost::mutex::scoped_lock lockguard(_refPublisherLock);
  if (_refPublisherSubscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "A new subscriber connected to the /" << NameOftopic_ref_
                                             << " ROS topic. The path plotter plugin will not publish messages in this "
                                                "topic.");
  }
  _refPublisherSubscribers++;
}

void PathPlotter::OnPublisherPathRefDisconnect()
{
  boost::mutex::scoped_lock lockguard(_refPublisherLock);
  _refPublisherSubscribers--;
  if (_refPublisherSubscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The last subscriber disconnected from the /" << NameOftopic_ref_
                                             << " ROS topic. The path plotter plugin will no longer publish messages "
                                                "in this "
                                                "topic.");
  }
  else if (_refPublisherSubscribers < 0)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error on the path ref publisher topic subscriber update. A negative "
                                                 "subscriber count was detecetd.");
  }
}

void PathPlotter::OnPublisherMarkerConnect()
{
  boost::mutex::scoped_lock lockguard(_markerPublisherLock);
  if (_markerPublisherSubscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "A new subscriber connected to the /" << NameOftopic_mark_
                                             << " ROS topic. The path plotter plugin will not publish messages in this "
                                                "topic.");
  }
  _markerPublisherSubscribers++;
}

void PathPlotter::OnPublisherMarkerDisconnect()
{
  boost::mutex::scoped_lock lockguard(_markerPublisherLock);
  _markerPublisherSubscribers--;
  if (_markerPublisherSubscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The last subscriber disconnected from the /" << NameOftopic_mark_
                                             << " ROS topic. The path plotter plugin will no longer publish messages "
                                                "in this "
                                                "topic.");
  }
  else if (_markerPublisherSubscribers < 0)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error on the path ref publisher topic subscriber update. A negative "
                                                 "subscriber count was detecetd.");
  }
}

GZ_REGISTER_MODEL_PLUGIN(PathPlotter)
}  // namespace gazebo
