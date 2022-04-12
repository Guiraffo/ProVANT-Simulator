/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file grid_ctrl.h
 * @brief This file contains the declaration of the GridCtrlPlugin class.
 *
 * The GuiCtrlPlugin that allows the control of the length and cell count of the
 * visual grid in the Gazebo GUI.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include <gazebo/gui/GuiPlugin.hh>

// If transport is necessary remove this line and the comment from the next ones
#ifndef Q_MOC_RUN
//#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#endif

#include <optional>

namespace gazebo
{
class GAZEBO_VISIBLE GridCtrlPlugin : public GUIPlugin
{
  Q_OBJECT
public:
  GridCtrlPlugin();
  virtual ~GridCtrlPlugin();
  void Load(sdf::ElementPtr _sdf) override;

protected:
  void SetupUi();
  void SetupGrid();

private:
  event::ConnectionPtr _renderingConn;

  const float DEFAULT_CELL_LENGTH = 1.0f;
  const float DEFAULT_HEIGHT_OFFSET = 0.0f;
  const unsigned int DEFAULT_CELL_COUNT = 20;
  const unsigned int DEFAULT_HEIGHT_COUNT = 1;
  const ignition::math::Color DEFAULT_COLOR{ 0.3f, 0.3f, 0.3f, 0.5f };

  std::optional<bool> _enabled;
  std::optional<float> _cellLength;
  std::optional<float> _gridHeightOffset;
  std::optional<unsigned int> _cellCount;
  std::optional<unsigned int> _cellCountHeight;
  std::optional<ignition::math::Color> _gridColor;

  bool _creationRequested = false;

  std::string _logMsg;
};
}  // namespace gazebo
