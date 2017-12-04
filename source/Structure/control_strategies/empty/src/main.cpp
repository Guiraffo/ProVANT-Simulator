#include "Icontroller.hpp"
#include "simulator_msgs/Sensor.h"
#include <Eigen/Eigen>


class empty : public Icontroller
{
	private: Eigen::VectorXd Input;
	private: Eigen::VectorXd X;

	public: empty():Input(1),X(3)
	{}
	public: ~empty(){}
	public: void config(){}
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
	    static int myswitch = 0;
	    static double valor = 0.005;
	    
	    	
		simulator_msgs::Sensor msgstates;
		msgstates = arraymsg.values.at(1);	
		
		X << msgstates.values.at(0),
             msgstates.values.at(1),
             msgstates.values.at(2);
		
		/*X << msgstates.values.at(24),//x
             msgstates.values.at(25),//y
             msgstates.values.at(26),//z
             msgstates.values.at(27),//roll
             msgstates.values.at(28),//pitch
             msgstates.values.at(29),//yaw
             msgstates.values.at(30),//vx
             msgstates.values.at(31),//vy
             msgstates.values.at(32),//vz
             msgstates.values.at(33),//ax
             msgstates.values.at(34),//ay
             msgstates.values.at(35),//az
             msgstates.values.at(12),//fx
             msgstates.values.at(13),//fy
             msgstates.values.at(14),//fz
             msgstates.values.at(39),//wx
             msgstates.values.at(40),//wy
             msgstates.values.at(41),//wz
             msgstates.values.at(42),//dwx
             msgstates.values.at(43),//dwy
             msgstates.values.at(44),//dwz
             msgstates.values.at(21),//tx
             msgstates.values.at(22),//ty
             msgstates.values.at(23);//tz*/
		
		std::vector<double> out(1);
		if(myswitch % 100 == 0)
		{
			valor = -valor;
		}
		out.at(0) = valor;
		//out.at(1) = 0;
		//out.at(2) = 0;
		//out.at(3) = 0;
		//out.at(4) = 0;
		//out.at(5) = 0;
		myswitch++;
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
		std::vector<double> out(X.data(), X.data() + X.rows() * X.cols());
		//std::vector<double> out;
		return out;
	}
};


extern "C"
{
	Icontroller *create(void) {return new empty;}
	void destroy(Icontroller *p) {delete p;}
}
