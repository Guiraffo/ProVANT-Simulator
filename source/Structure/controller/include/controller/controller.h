/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * \file This file contains the declaration of the Controller2 class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <map>
#include <string>
#include <dlfcn.h>
#include <fstream>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "controller/MatlabData.h"
#include "controller/XMLRead.h"
#include "simulator_msgs/Sensor.h"
#include "simulator_msgs/SensorArray.h"
#include "Icontroller.hpp"

class ControllerNode
{
public:
	static void init(int argc, char** argv);

	ControllerNode();
	virtual ~ControllerNode();

  void Start();
	void configPrint();

private: 
  std::map<std::string, int> mapOfWords;
	boost::mutex mtx;
	ros::NodeHandle nh;
	std::vector<ros::Subscriber> subarray;
	int cont;
	simulator_msgs::SensorArray data;

	MatlabData outfile;
	MatlabData infile;
	MatlabData reffile;
	MatlabData erfile;
	std::vector<double> out;
	std::vector<ros::Publisher> pubarray;
	ros::Publisher Step_pub;
	ros::Subscriber X_sub;
	int decimador;
	void* teste = NULL;
	Icontroller* controller = NULL;
	int T;
	XMLRead docmem;

	void Sensor(simulator_msgs::Sensor msg);
	void control_law(simulator_msgs::SensorArray msg);
	void Step();
	void configPlugin();
};

#endif // CONTROLLER_H
