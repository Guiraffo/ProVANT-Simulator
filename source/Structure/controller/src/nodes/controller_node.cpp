/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the entry point for the controller node.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <iostream>
#include "controller/controller.h"

/**
 * @brief Entry point for the controller package node.
 * 
 * @param argc Number of received arguments.
 * @param argv Vector of received arguments as strings.
 * @return int Execution status. Returns 0 in case of success and a negative
 * integer otherwise.
 */
int main (int argc, char **argv)
{
	// Initialize ROS
	ControllerNode::init(argc, argv);
  // Create an instance of the Controller2 class and configure it.
	ControllerNode Instance;
	Instance.startSimulation();

  // Keep this node running until ROS finishes execution.
	while(ros::ok())
    ros::spin();

	return 0;
}
