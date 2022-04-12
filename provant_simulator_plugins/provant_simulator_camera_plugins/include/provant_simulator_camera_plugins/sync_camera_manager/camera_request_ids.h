/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file camera_request_ids.h
 * @brief This file contains the constants defining the names of the valid
 * requests and responses used in the Gazebo tranport based communication of
 * the ProVANT Simulator Camera plugins.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CAMERA_REQUEST_IDS_H
#define PROVANT_CAMERA_REQUEST_IDS_H

#include <string>

namespace provant::camera_plugins
{
/**
 * @brief Contains the ids of the requests used in the gazebo tranport communication used by the ProVANT Camera plugins.
 */
namespace request_names
{
//! The namespace used to uniquely identify the requests used in the ProVANT camera plugins.
const std::string REQ_NAMESPACE = "provant_sync_camera/";
//! Request sent from the recorder to the manager plugin used to enable a synchronized recorder.
const std::string ENABLE_RECORDER = REQ_NAMESPACE + "enable";
//! Request sent from the recorder to the manager plugin used to disable a synchronized recorder.
const std::string DISABLE_RECORDER = REQ_NAMESPACE + "disable";
//! Request sent from the manager to the recorder plugin used to inform the recorder that a new frame must be saved.
const std::string GRAB_FRAME = REQ_NAMESPACE + "grab_frame";
//! Request sent from the manager to the recorder plugin used to request an indication about when the recorder must be
//! enabled.
const std::string ENABLE_ON = REQ_NAMESPACE + "enable_on";
}  // namespace request_names

/**
 * @brief Constains the possible response status used in the gazebo tranport communication used by the ProVANT Camera
 * plugins.
 */
namespace response_status
{
const std::string SUCCESS = "success";
const std::string ERROR = "error";
}  // namespace response_status

}  // namespace provant::camera_plugins

#endif  // PROVANT_CAMERA_REQUEST_NAMES_H
