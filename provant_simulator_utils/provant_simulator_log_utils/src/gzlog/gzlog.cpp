/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file gzlog.cpp
 * @brief This file contains the implemantion of the GazeboSinkBase class.
 *
 * For more detailed documentation check the gzlog.h header.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_log_utils/gzlog/gzlog.h"

using provant::log::GazeboLogger;

GazeboLogger::GazeboLogger(std::string name) : spdlog::logger(name, std::make_shared<GazeboLogSink>(name))
{
}
