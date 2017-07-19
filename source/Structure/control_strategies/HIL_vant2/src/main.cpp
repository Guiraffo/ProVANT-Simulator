#include "Icontroller.hpp"
#include "protocolo.hpp"


class HIL_vant2 : public Icontroller
{
	private: int start;
		 std::vector<double> in2;

	private: protocolo uart;

	public: HIL_vant2():uart("/dev/ttyUSB0"),in2(20){ start = 0;}
	public: ~HIL_vant2(){}
	public: void config(){}
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		if(start==0)
		{
			std::cout << "sincronizando" << std::endl;
			uart.sincronization();
			start = 1;
		}
		
		const clock_t begin_time = clock();

		//std::cout << "Entrou" << std::endl;
		simulator_msgs::Sensor msgstates;
		msgstates = arraymsg.values.at(0);
		std::vector<float> in(msgstates.values.begin(), msgstates.values.end());
		
		for(int i=0;i<in.size();i++) 
		in2.at(i) = (double)in.at(i);		

		/*std::cout << "Dados entrada2" << std::endl;
		for(int i=0;i<in.size();i++) std::cout << in.at(i) << "  ";
		std::cout << "\n";*/
		uart.sendData(in,20);
		std::vector<float> out(4);
		uart.waitAndreceiveData2(&out,4);
		/*std::cout << "Dados saida" << std::endl;
		for(int i=0;i<out.size();i++) std::cout << out.at(i) << "  ";
		std::cout << "\n";*/
		std::vector<double> doubleout(4);
		doubleout.at(0) = (double)out.at(0);
		doubleout.at(1) = (double)out.at(1);
		doubleout.at(2) = (double)out.at(2);
		doubleout.at(3) = (double)out.at(3);

		// do something
		std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
		
		//std::cout << "Saiu" << std::endl;
		return doubleout;
	}
	public: std::vector<double> Reference()
	{
		std::vector<double> out(1);
		return out;
	}
	public: std::vector<double> Error()
	{
		std::vector<double> out(1);
		return out;
	}
	public: std::vector<double> State()
	{
		return in2;
	}
};


extern "C"
{
	Icontroller *create(void) {return new HIL_vant2;}
	void destroy(Icontroller *p) {delete p;}
}
