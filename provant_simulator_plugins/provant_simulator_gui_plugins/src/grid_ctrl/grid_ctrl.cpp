/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file grid_ctrl.h
 * @brief This file contains the implementation of the GridCtrlPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "grid_ctrl.h"

#include <provant_simulator_sdf_parser/sdf_parser.h>

using namespace gazebo;

GridCtrlPlugin::GridCtrlPlugin() : GUIPlugin()
{
  SetupUi();
}

GridCtrlPlugin::~GridCtrlPlugin()
{
}

void GridCtrlPlugin::Load(sdf::ElementPtr _sdf)
{
  _logMsg = "[GridCtrlPlugin: " + GetHandle() + "] ";
  gzmsg << _logMsg << "Starting the plugin loading process.\n";

  SDFParser parser(_sdf);

  // Check if the enabled element was specified
  try
  {
    if (parser.HasElement("enabled"))
    {
      _enabled.emplace(parser.GetElementBool("enabled"));
      if (_enabled.value())
        gzdbg << _logMsg << "The grid is enabled.\n";
      else
        gzdbg << _logMsg << "The grid is disaled.\n";
    }
    if (parser.HasElement("cell_length"))
    {
      _cellLength.emplace(parser.GetElementFloat("cell_length"));
      gzdbg << _logMsg << "The grid wil have a cell length of " << _cellLength.value() << " meter(s).\n";
    }
    if (parser.HasElement("cell_count"))
    {
      _cellCount.emplace(parser.GetElementUnsignedInt("cell_count"));
      gzdbg << _logMsg << "The grid will have " << _cellCount.value() << " cell(s) in the X and Y directions.\n";
    }
    if (parser.HasElement("grid_height"))
    {
      _cellCountHeight.emplace(parser.GetElementUnsignedInt("grid_height"));
      gzdbg << _logMsg << "The grid will have " << _cellCountHeight.value() << " cell(s) in the Z direction.\n";
    }
    if (parser.HasElement("height_offset"))
    {
      _gridHeightOffset.emplace(parser.GetElementFloat("height_offset"));
      gzdbg << _logMsg << "The grid will have a height offset of " << _gridHeightOffset.value() << " meters.\n";
    }
    if (parser.HasElement("color"))
    {
      auto colorParser = SDFParser(_sdf->GetElement("color"));

      const auto r = std::max(0U, std::min(colorParser.GetElementUnsignedInt("r"), 255U));
      const auto g = std::max(0U, std::min(colorParser.GetElementUnsignedInt("g"), 255U));
      const auto b = std::max(0U, std::min(colorParser.GetElementUnsignedInt("b"), 255U));
      unsigned int a = 255;
      if (colorParser.HasElement("a"))
      {
        a = std::max(0U, std::min(colorParser.GetElementUnsignedInt("a"), 255U));
      }
      gzdbg << _logMsg << "The grid will have the (" << r << "," << g << "," << b << "," << a << ") RGBA color.\n";

      const auto f_r = static_cast<float>(r) / 255.0f;
      const auto f_g = static_cast<float>(g) / 255.0f;
      const auto f_b = static_cast<float>(b) / 255.0f;
      const auto f_a = static_cast<float>(a) / 255.0f;

      _gridColor.emplace(f_r, f_g, f_b, f_a);
    }
  }
  catch (const SDFStatus& e)
  {
    gzerr << _logMsg << "An exception was caught while parsing the plugin SDF configuration with message: " << e.what()
          << '\n';
    return;
  }

  // Setup connection
  gzdbg << _logMsg << "Connecting to the Rendering signal.\n";
  _renderingConn = event::Events::ConnectRender([this]() { this->SetupGrid(); });

  gzmsg << _logMsg << "The plugin was successfully loaded.\n";
}

void GridCtrlPlugin::SetupUi()
{
  move(0, 0);
  resize(0, 0);
  setVisible(false);
  hide();
}

void GridCtrlPlugin::SetupGrid()
{
  auto scene = rendering::get_scene();

  if (!scene || !scene->Initialized())
    return;

  /*
   * If no scene exists, but one should be created (the enabled value is true), request the creation of scene.
   * Note: As the constructor doesn't allow the creation of a grid specifying the height count and height offset values,
   * we need to keep running this method until the grid has been created, and we can set this values.
   */
  if (scene->GridCount() == 0 && !_creationRequested)
  {
    // If no scene exists, check the value of the enabled parameter
    if (_enabled.value_or(true))
    {
      gzdbg << _logMsg << "Creating a new grid.\n";
      // Create a scene with the default values
      scene->CreateGrid(_cellCount.value_or(DEFAULT_CELL_COUNT), _cellLength.value_or(DEFAULT_CELL_LENGTH),
                        _gridColor.value_or(DEFAULT_COLOR));

      // Check if the user specified values for the height cell count and offset.
      if ((_cellCountHeight.has_value() && _cellCountHeight.value() != DEFAULT_HEIGHT_COUNT) ||
          (_gridHeightOffset.has_value() && _gridHeightOffset.value() != DEFAULT_HEIGHT_OFFSET))
      {
        _creationRequested = true;
      }
      else
      {
        gzmsg << _logMsg
              << "Grid created according to the requested values. Releasing connection to the rendering signal.\n";
        _renderingConn.reset();
      }

      return;
    }
    else
    {
      // As no scene exists, and no scene should be created, there are no tasks to be handled by the plugin, so we can
      // release the connection to render.
      gzmsg << _logMsg
            << "No grid exists and the grid creation is disabled. Releasing connection to the rendering signal.\n";
      _renderingConn.reset();
      return;
    }
  }

  /*
   * If no grid exists, return and wait until at least one grid exists.
   * Note: It is only possible to reach this point if the creation of a grid was requested, and this grid still hasn't
   * been created.
   */
  if (scene->GridCount() == 0)
    return;

  // Updates the values of all grids
  for (uint32_t i = 0; i < scene->GridCount(); ++i)
  {
    gzdbg << _logMsg << "Updating the values of the " << i + 1 << "-th grid.\n";
    if (_enabled.has_value() && !_enabled.value())
    {
      scene->GetGrid(i)->Enable(false);
    }
    if (_cellLength.has_value())
    {
      scene->GetGrid(i)->SetCellLength(_cellLength.value());
    }
    if (_cellCount.has_value())
    {
      scene->GetGrid(i)->SetCellCount(_cellCount.value());
    }
    if (_gridColor.has_value())
    {
      scene->GetGrid(i)->SetColor(_gridColor.value());
    }
    if (_cellCountHeight.has_value())
    {
      scene->GetGrid(i)->SetHeight(_cellCountHeight.value());
    }
    if (_gridHeightOffset.has_value())
    {
      scene->GetGrid(i)->SetHeightOffset(_gridHeightOffset.value());
    }
  }

  // At this point, there is no more work to be done, release the connection
  gzmsg << _logMsg << "Configuration successfully completed. Releasing connection to the rendering signal.\n";
  _renderingConn.reset();
}

GZ_REGISTER_GUI_PLUGIN(GridCtrlPlugin)
