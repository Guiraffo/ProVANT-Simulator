/*
* File: tilt_controller2.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This file is the startup file of the UAV model's control software
*/


#include "controller/Controller2.cpp"
#include <exception>


int main (int argc, char **argv)
{
	try
	{
		// Call a header and starts it
		Controller2::init(argc,argv);
		Controller2 Instance;
		Instance.Start();
		return 0;
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
}


