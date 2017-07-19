//#ifndef MATLABDATA_HPP
//#define MATLABDATA_HPP
class MatlabData
{
	private: std::fstream file;	
	
	public: MatlabData()
	{

	}	
	public: ~MatlabData()
	{

	}	
	public: void startFile(std::string namefile, std::string varname)
	{
		file.open (namefile.c_str(), std::fstream::out);
		//file << varname << " = [";
	}
	
	public: void endFile()
	{
		if (file.is_open())
		{
			//file << "];";
			file.close();
		}
	}

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
//#endif
