#ifndef XMLREAD
#define XMLREAD
class XMLRead
{
	public: static std::string ReadXMLString(std::string input,sdf::ElementPtr _sdf)
	{
		if (_sdf->HasElement(input)) return(_sdf->GetElement(input)->Get<std::string>());
	  	else 
	  	{  
			std::cout << "Coloque <"+input+"> x </"+input+">" << std::endl;
			exit(1);
	  	}
	}
	
	public: static double ReadXMLDouble(std::string input,sdf::ElementPtr _sdf)
	{
		if (_sdf->HasElement(input)) return(_sdf->GetElement(input)->Get<double>());
	  	else 
	  	{  
			std::cout << "Coloque <"+input+"> x </"+input+">" << std::endl;
			exit(1);
	  	}
	}
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
