#include <exception>
#include <iostream>


int main (int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "server");
		ros::NodeHandle n;
		ros::Publisher chatter_pub = n.advertise<std_msgs::String>("teste", 1000);
		
		return 0;
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return 1;
	}
}


