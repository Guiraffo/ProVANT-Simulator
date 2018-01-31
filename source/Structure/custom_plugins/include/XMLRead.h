/*
* File: XMLRead.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement code to read some data from SDF file
*/

#ifndef XMLREAD
#define XMLREAD



class XMLRead
{
	// Read String
	public: static std::string ReadXMLString(std::string input,sdf::ElementPtr _sdf)
	{
		if (_sdf->HasElement(input)) return(_sdf->GetElement(input)->Get<std::string>());
	  	else 
	  	{  
			std::cout << "Coloque <"+input+"> x </"+input+">" << std::endl;
			exit(1);
	  	}
	}
	
	// Read Double
	public: static double ReadXMLDouble(std::string input,sdf::ElementPtr _sdf)
	{
		if (_sdf->HasElement(input)) return(_sdf->GetElement(input)->Get<double>());
	  	else 
	  	{  
			std::cout << "Coloque <"+input+"> x </"+input+">" << std::endl;
			exit(1);
	  	}
	}
	
	// Read Integer	
	public: static int ReadXMLInt(std::string input,sdf::ElementPtr _sdf)
	{
		if (_sdf->HasElement(input)) return(_sdf->GetElement(input)->Get<int>());
	  	else 
	  	{  
			std::cout << "Coloque <"+input+"> x </"+input+">" << std::endl;
			exit(1);
	  	}
	}		
};
#endif
