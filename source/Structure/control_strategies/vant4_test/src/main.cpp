#include "Icontroller.hpp"
#include "ros/ros.h"
#include <cstdlib>


class vant4_test : public Icontroller
{
	int count;
	ros::NodeHandle n;

	enum
	{
		thrust_R, thrust_L,
		rotor_R, rotor_L,
		rudder_R, rudder_L
	};

	public: vant4_test() : count(0) {}
	public: ~vant4_test(){}
	public: void config(){}
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		std::vector<double> out(6);

		out.at(rudder_R) = n.param("vant4/rudder_R",(abs(++count % 100 - 50) - 25) * 3.14159/100);
		out.at(rudder_L) = n.param("vant4/rudder_L",-(abs(++count % 100 - 50) - 25) * 3.14159/100);

		out.at(thrust_R) = n.param("vant4/thrust_R",50);
		out.at(thrust_L) = n.param("vant4/thrust_L",50);

	if(n.param("vant4/control",false))
	{
		double pitch = arraymsg.values.at(0).values.at(4);
		double x = arraymsg.values.at(0).values.at(0);
		while(pitch > 3.14159)
			pitch -= 3.14159;

		while(pitch < -3.14159)
			pitch += 3.14159;

		double k_theta = n.param("vant4/k_theta",1);
		double k_x = n.param("vant4/k_x",1);

		out.at(rotor_L) = out.at(rotor_R) = k_theta*pitch + k_x*x;
	}
	else
	{
		out.at(rotor_R) = n.param("vant4/rotor_R",(abs(++count % 100 - 50) - 25) * 3.14159/100);
		out.at(rotor_L) = n.param("vant4/rotor_L",-(abs(++count % 100 - 50) - 25) * 3.14159/100);
	}

		return out;
	}

	public: std::vector<double> Reference()
	{
		std::vector<double> out;
		return out;
	}
	public: std::vector<double> Error()
	{
		std::vector<double> out;
		return out;
	}
	public: std::vector<double> State()

	{
		std::vector<double> out;
		return out;
	}
};


extern "C"
{
	Icontroller *create(void) {return new vant4_test;}
	void destroy(Icontroller *p) {delete p;}
}
