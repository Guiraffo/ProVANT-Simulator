/*
* File: MatlabData.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This library is responsable to implement code to print some data in a file with Matlab form
*/

#include <fstream>

class MatlabData
{
	private: std::fstream file;	
	
	// constructor
	public: MatlabData()
	{

	}
	// destructor	
	public: ~MatlabData()
	{

	}
	// open file to print data with desired name
	public: bool startFile(std::string namefile, std::string varname)
	{
		file.open (namefile.c_str(), std::fstream::out);
		return file.good();
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
	public: void printFile(std::vector<double> data)
	{
		if(file.is_open())
		{	
			if(data.size()>0)
			{
				file << data.at(0);
				int i = 1;
				while(i<data.size())
				{		
				file <<","<<data.at(i);
				i++;
				}
				file << std::endl;
			}
		}
	}
};
