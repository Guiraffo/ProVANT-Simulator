#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

#include "simulator_msgs/SensorArray.h"

class Icontroller 
{
	public:
	Icontroller(){};
	virtual ~Icontroller(){};
	virtual void config()=0;
	virtual std::vector<double> execute(simulator_msgs::SensorArray)=0;
	virtual std::vector<double> Reference()=0;
	virtual std::vector<double> Error()=0;
	virtual std::vector<double> State()=0;
};

extern "C" {
typedef Icontroller* create_t();
typedef void destroy_t(Icontroller*);
}
#endif

