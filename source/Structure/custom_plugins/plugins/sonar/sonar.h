#ifndef _GAZEBO_SONAR_PLUGIN_HH_
#define _GAZEBO_SONAR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>
#include "ros/ros.h"
#include <random>
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"


namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class sonar : public ModelPlugin
  {
    /// \brief Constructor.
    public: sonar();

    /// \brief Destructor.
    public: virtual ~sonar();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate(ConstSonarPtr&);

     physics::WorldPtr world;

    //private: ros::NodeHandle n;

    //private: ros::Publisher gps_pub;

    private: std::string gazebotopic;
    private: std::string rostopic;
    private: transport::SubscriberPtr sub;
    private: transport::NodePtr node;
    physics::ModelPtr model;

    ros::NodeHandle n;
    ros::Publisher publisher_;

	double mm2uS;

  };
}
#endif
