#include "controller/Controller2.cpp"
#include <exception>


int main (int argc, char **argv)
{
	try
	{
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


