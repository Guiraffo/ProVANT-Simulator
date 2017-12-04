#include "Icontroller.hpp"


class teste : public Icontroller
{
	public: teste(){}
	public: ~teste(){}
	public: void config(){}
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		std::vector<double> out;
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
	Icontroller *create(void) {return new teste;}
	void destroy(Icontroller *p) {delete p;}
}
