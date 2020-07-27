/*
* File: MatlabData.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This library is responsable to implement code to print some data in a file with Matlab form
*/

class testedois
{
	private: std::fstream file;	
	
	// constructor
	public: testedois()
	{

	}
	// destructor	
	public: ~testedois()
	{

	}
	// open file to print data with desired name
	public: void startFile(std::string namefile, std::string varname)
	{
		file.open (namefile.c_str(), std::fstream::out);
	}
	
	// close file
	public: void endFile()
	{
		if (file.is_open())
		{
			file.close();
		}
	}
	
	// print data
	public: void printFile(double data)
	{
		if(file.is_open())
		{	
		file << data;
		file <<std::endl;
		//	if(data.size()>0)
		//	{
		//		file << data.at(0);
		//		int i = 1;
		//		while(i=data.size())
		//		{		
		//		file <<","<<data.at(0);
		//		i++;
		//		}
		//		file << std::endl;
		//	} 
		}
	}
};
