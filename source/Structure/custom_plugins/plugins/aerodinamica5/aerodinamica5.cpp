/*
 * File: aerodinamica5.cpp
 * Author: Jonatan Mota Campos
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 14/10/2020
 * Description:  This library is responsable to implement aerodynamics forces in a UAV of type tilt rotor with four
 * rotors
 */

#include <aerodinamica5.h>

namespace gazebo
{
/*****************************************************************************************************************************/
// constructor
aerodinamica5::aerodinamica5()

{
}

/*****************************************************************************************************************************/

// Destructor
aerodinamica5::~aerodinamica5()
{  // Wind.endFile();
  try
  {
    //	updateTimer.Disconnect(updateConnection);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

// to load initial setup
void aerodinamica5::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "vant 5.0 aerodynamics initialized" << std::endl;
  try
  {
    if (!ros::isInitialized())
    {
      std::cout << "aerodinamica5 not initialized!" << std::endl;
      return;
    }

    topic_FR2 = XMLRead::ReadXMLString("topic_FR2", _sdf);  // name of brushless number 1
    topic_FL3 = XMLRead::ReadXMLString("topic_FL3", _sdf);  // name of brushless number 2
    topic_FR4 = XMLRead::ReadXMLString("topic_FR4", _sdf);  // name of brushless number 3
    topic_FL5 = XMLRead::ReadXMLString("topic_FL5", _sdf);  // name of brushless number 4
    NameOfLink2_ = XMLRead::ReadXMLString("Rotor2", _sdf);  // name of rotor2 link
    NameOfLink3_ = XMLRead::ReadXMLString("Rotor3", _sdf);  // name of rotor3 link
    NameOfLink4_ = XMLRead::ReadXMLString("Rotor4", _sdf);  // name of rotor4 link
    NameOfLink5_ = XMLRead::ReadXMLString("Rotor5", _sdf);  // name of rotor5 link
    DragConst = XMLRead::ReadXMLDouble("DragCte", _sdf);    // Drag constant

    // get elements of the simulation
    linkFr2 = _model->GetLink(NameOfLink2_);
    linkFl3 = _model->GetLink(NameOfLink3_);
    linkFr4 = _model->GetLink(NameOfLink4_);
    linkFl5 = _model->GetLink(NameOfLink5_);
    // get world pointer
    world = _model->GetWorld();

    // update timer
    //		Reset();
    //		updateTimer.Load(world, _sdf);
    //	updateConnection = updateTimer.Connect(boost::bind(&aerodinamica5::Update, this));

    // subscribers of data to apply in simulator
    motor_subscriberFR2_ = node_handle_.subscribe(topic_FR2, 1, &gazebo::aerodinamica5::CallbackFR2, this);
    motor_subscriberFL3_ = node_handle_.subscribe(topic_FL3, 1, &gazebo::aerodinamica5::CallbackFL3, this);
    motor_subscriberFR4_ = node_handle_.subscribe(topic_FR4, 1, &gazebo::aerodinamica5::CallbackFR4, this);
    motor_subscriberFL5_ = node_handle_.subscribe(topic_FL5, 1, &gazebo::aerodinamica5::CallbackFL5, this);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

/*void aerodinamica5::Reset()
{
  try
  {
    updateTimer.Reset();
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}
*/

/*****************************************************************************************************************************/

// callback to get the force at brushless 1
void aerodinamica5::CallbackFR2(std_msgs::Float64 msg)
{
  try
  {
    ignition::math::Vector3d forceR2(0, 0, msg.data);
    //	ignition::math::Vector3d offset2(0, 0, -0.00457);
    linkFr2->AddRelativeForce(forceR2);
    //	linkFr2->AddLinkForce(forceR2,offset2);

    //  ignition::math::Vector3d torqueR2(0,0,0.0178947368*msg.data); // drag torque
    // Applying
    //		linkFr2->AddRelativeTorque(torqueR2);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

// callback to get the force at brushless of rotor 3
void aerodinamica5::CallbackFL3(std_msgs::Float64 msg)
{
  try
  {
    ignition::math::Vector3d forceL3(0, 0, msg.data);
    //	ignition::math::Vector3d offset3(0, 0, -0.00457);

    linkFl3->AddRelativeForce(forceL3);
    //	linkFl3->AddLinkForce(forceL3,offset3);

    //    ignition::math::Vector3d torqueL3(0,0,0.0178947368*msg.data); // drag torque
    // Applying
    //    linkFl3->AddRelativeTorque(torqueL3);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

// callback to get the force at brushless 3
void aerodinamica5::CallbackFR4(std_msgs::Float64 msg)
{
  try
  {
    ignition::math::Vector3d forceR4(0, 0, msg.data);
    //	ignition::math::Vector3d offset4(-0.01769, 0, -0.08356);
    linkFr4->AddRelativeForce(forceR4);
    //	linkFr4->AddLinkForce(forceR4,offset4);

    //    ignition::math::Vector3d torqueR4(0,0,0.0178947368*msg.data); // drag torque
    // Applying
    //      linkFr4->AddRelativeTorque(torqueR4);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

// callback to get the force at brushless 4
void aerodinamica5::CallbackFL5(std_msgs::Float64 msg)
{
  try
  {
    ignition::math::Vector3d forceL5(0, 0, msg.data);
    //	ignition::math::Vector3d offset5(-0.01769, 0, -0.08356);
    linkFl5->AddRelativeForce(forceL5);
    //		linkFl5->AddLinkForce(forceL5,offset5);
    //    ignition::math::Vector3d torqueL5(0,0,0.0178947368*msg.data); // drag torque
    // Applying
    //    linkFl5->AddRelativeTorque(torqueL5);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

GZ_REGISTER_MODEL_PLUGIN(aerodinamica5)
}  // namespace gazebo
