/*
* File: Icontroller.hpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This file is responsable to implement interface of dynamics libraries with control laws
*/

#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

#include "simulator_msgs/SensorArray.h"

class Icontroller 
{
	public:
	// constructor
	Icontroller(){};
	// destructor
	virtual ~Icontroller(){};
	// method built to make initial setup
	virtual void config()=0;
	// method built to implement control law. It is called every Sampled time (argument = Sensor Data)
	virtual std::vector<double> execute(simulator_msgs::SensorArray)=0;
	// method built to allow user show reference data to be printed
	virtual std::vector<double> Reference()=0;
	// method built to allow user show error data to be printed
	virtual std::vector<double> Error()=0;
	// method built to allow user show state data to be printed
	virtual std::vector<double> State()=0;
};


// to implement dynamics libraries
extern "C" {
typedef Icontroller* create_t();
typedef void destroy_t(Icontroller*);
}
#endif

