#include "controller/Controller.cpp"
#include <exception>


int main (int argc, char **argv)
{
	try
	{
		Controller::init(argc,argv);
		Controller Instance;
		Instance.Start();
		return 0;
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
}


