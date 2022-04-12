/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the PathPlotter class.
 *
 * @author Jonatan Mota Campos
 */

#ifndef PATH_PLOTTER_H
#define PATH_PLOTTER_H

#include <gazebo/common/Plugin.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
//#include <sensor_msgs/LaserScan.h>

#include <boost/thread.hpp>

#include <string>

namespace gazebo
{
/**
 * @brief The PathPlotter class is a Gazebo plugin that publishes the necessary
 * information for the visualization of the UAV reference trajectory and current
 * pose in RViz, allowing the creation of visually interesting plots for
 * publications and visual feedback of the simulations.
 */
class PathPlotter : public ModelPlugin
{
public:
  /**
   * @brief Construct a new Path Plotter object.
   */
  PathPlotter() = default;
  /**
   * @brief Destroy the Path Plotter object.
   * Close the ROS publishers and the connection to the WorldUpdateEnd event.
   */
  virtual ~PathPlotter() = default;
  /**
   * @brief Method called when Gazebo loads this plugin.
   *
   * Reads the necessary parameters from the plugin SDF, advertises the ROS
   * topics used in the simulation and configures the simulation.
   *
   * @param _model Pointer to the model which contains this plugin.
   * @param _sdf Pointer to the SDF element which instantiated this plugin.
   */
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Method called at every simulation step.
   *
   * Checks if there are any active subscribers to the actual UAV pose, the
   * reference UAV pose, and the UAV marker, and also broadcasts the
   * transformations with the UAV pose, and reference pose.
   */
  virtual void Update();

  /**
   * @brief Method called when a new subscriber connects to the publisher_path_ topic.
   *
   * Increments the subscriber count.
   */
  virtual void OnPublisherPathConnect();
  /**
   * @brief Method called when a subscriber disconnects from the publisher_path_ topic.
   *
   * Decrements the subscriber count.
   */
  virtual void OnPublisherPathDisconnect();
  /**
   * @brief Method called when a new subscriber connects to the publisher_path_ref_ topic.
   *
   * Increments the subscriber count.
   */
  virtual void OnPublisherPathRefConnect();
  /**
   * @brief Method called when a subscriber disconnects from the publisher_path_ref_ topic.
   *
   * Decrements the subscriber count.
   */
  virtual void OnPublisherPathRefDisconnect();
  /**
   * @brief Method called when a new subscriber connects to the publisher_marker_ topic.
   *
   * Increments the subscriber count.
   */
  virtual void OnPublisherMarkerConnect();
  /**
   * @brief Method called when a subscriber disconnects from the publisher_marker_ topic.
   *
   * Decrements the subscriber count.
   */
  virtual void OnPublisherMarkerDisconnect();

private:
  //! Name of the topic used to publish the actual UAV pose.
  std::string NameOftopic_path_;
  //! Name of the topic used to publish the marker for the UAV visualization.
  std::string NameOftopic_mark_;
  //! Name of the topic used to publish the reference pose.
  std::string NameOftopic_ref_;
  //! Name of the link that composes the UAV main body, from where the pose will be read.
  std::string link_name_;
  //! Indicates which UAV is being used.
  std::string tag_;
  //! Pointer to the UAV main body link.
  physics::LinkPtr link;
  //! Pointer to the simulation world.
  physics::WorldPtr world;
  //! Pointer to the connection used to run the Update method every simulation step.
  event::ConnectionPtr updateConnection;
  //! ROS Node handle that manages the topics advertised by this node.
  ros::NodeHandle node_handle_;
  //! Mutex used to ensure the Update method is only executed by one thread at a time.
  boost::mutex lock;
  //! ROS publisher to send the actual UAV pose.
  ros::Publisher publisher_path_;
  //! ROS publisher to send the reference UAV pose.
  ros::Publisher publisher_path_ref_;
  //! ROS publisher to publish the marker with the UAV position.
  ros::Publisher publisher_marker_;
  //! Transformation broadcaster for TF version 1.
  tf::TransformBroadcaster broadcaster;
  //! Transformation broadcaster for TF version 2.
  tf2_ros::StaticTransformBroadcaster broadcaster_2;
  //! Message used to identify the logs of this plugin in the output screen.
  std::string _logMsg;
  //! Name of this plugin child logger.
  const std::string PLUGIN_ID = "path_plotter_plugin";
  //! Mutex used in the subscriber count update of the publisher_path_ topic.
  boost::mutex _pathPublisherLock;
  //! Number of subscribers connected to the publisher_path_ topic.
  int _pathPublisherSubscribers = 0;
  //! Mutex used in the subscriber count update of the publisher_path_ref_ topic.
  boost::mutex _refPublisherLock;
  //! Number of subscribers connected to the publisher_path_ref_ topic.
  int _refPublisherSubscribers = 0;
  //! Mutex used in the subscriber count update of the publisher_marker_ topic.
  boost::mutex _markerPublisherLock;
  //! Number of subscribers connected to the publisher_marker_ topic.
  int _markerPublisherSubscribers = 0;
};
}  // namespace gazebo

#endif  // PATH_PLOTTER_H
