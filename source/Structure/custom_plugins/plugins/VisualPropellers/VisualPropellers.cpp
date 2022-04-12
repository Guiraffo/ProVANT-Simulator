
/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the VisualPropellers class.
 *
 * @author Jonatan Mota Campos
 */

#include <VisualPropellers.h>

#include <gazebo/common/CommonTypes.hh>  // Used for logging

#include "XMLRead.h"

namespace gazebo
{
// Called when the plugin is loaded by Gazebo
void VisualPropellers::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  try
  {
    _logMsg = "[VisualPropellersPlugin: " + GetHandle() + "] ";

    gzmsg << _logMsg << "Starting plugin loading process.\n";

    _propeller1Name = XMLRead::ReadXMLString("Propeller1", _sdf);
    _propeller2Name = XMLRead::ReadXMLString("Propeller2", _sdf);
    _velocity = XMLRead::ReadXMLDouble("Propellers_Velocity", _sdf);

    _propeller1 = _model->GetJoint(_propeller1Name);
    _propeller2 = _model->GetJoint(_propeller2Name);

    // Verify if the propeller pointers are valid
    if (_propeller1 == nullptr)
    {
      gzerr << _logMsg << "Error, it was not possible to find a joint with name \"" << _propeller1Name << "\" in the "
            << _model->GetName() << " model. The VisualPropellersPlugin will not be loaded.\n";
      return;
    }
    if (_propeller2 == nullptr)
    {
      gzerr << _logMsg << "Error, it was not possible to find a joint with name \"" << _propeller2Name << "\" in the "
            << _model->GetName() << " model. The VisualPropellersPlugin will not be loaded.\n";
      return;
    }

    // Check if the velocity is under the specified limit for the joint
    const auto prop1Limit = _propeller1->GetVelocityLimit(0);
    const auto prop2Limit = _propeller2->GetVelocityLimit(0);

    if (_velocity < -prop1Limit || _velocity > prop1Limit)
    {
      gzerr << _logMsg << "Error, the specified velocity of " << _velocity << " rad/s is not within the limits of the "
            << _propeller1Name << " joint. The speed must be between " << -prop1Limit << " rad/s and " << prop1Limit
            << " rad/s.\n";
      return;
    }
    if (_velocity < -prop2Limit || _velocity > prop2Limit)
    {
      gzerr << _logMsg << "Error, the specified velocity of " << _velocity << " rad/s is not within the limits of the "
            << _propeller2Name << " joint. The speed must be between " << -prop2Limit << " rad/s and " << prop2Limit
            << " rad/s.\n";
      return;
    }

    // Setup a connection to set the velocities in each step time.
    gzmsg << _logMsg << "Connecting to the WorldUpdateBegin signal\n";
    _worldUpdateBeginConn = event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo& info) {
      (void)info;  // Silence unused variable warning
      this->OnWorldUpdateBegin();
    });
    if (!_worldUpdateBeginConn)
    {
      gzerr << _logMsg << "Error while connecting to the WorldUpdateBegin signal. The plugin cannot start execution.\n";
      return;
    }

    // Apply the velocities
    gzmsg << _logMsg << "Applying a velocity of " << _velocity << " rad/s at the " << _propeller1Name << " and "
          << _propeller2Name << " propellers.\n";
    _propeller1->SetVelocity(0, _velocity);
    _propeller2->SetVelocity(0, _velocity);

    gzmsg << _logMsg << "Plugin loaded successfully.\n";
  }
  catch (const std::exception& e)
  {
    gzerr << _logMsg << "An unexpected exception occurred at the Load method with message: \"" << e.what() << "\".\n";
  }
}

// Called when the simulation is reseted
void VisualPropellers::Reset()
{
  gzmsg << _logMsg << "Resetting the plugin. Applying a velocity of " << _velocity << " rad/s at the "
        << _propeller1Name << " and" << _propeller2Name << " propellers.\n";
}

void VisualPropellers::OnWorldUpdateBegin()
{
  _propeller1->SetVelocity(0, _velocity);
  _propeller2->SetVelocity(0, _velocity);
}

GZ_REGISTER_MODEL_PLUGIN(VisualPropellers)
}  // namespace gazebo
