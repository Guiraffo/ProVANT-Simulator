#include <string>


extern "C"
{
	#include "serial.h"
}
#include <vector>
#define MAX_SIZE_DATA 100


class protocolo
{
	public: int fd;

	public: protocolo(std::string);
		~protocolo();
		void sincronization();
		void sendData(std::vector<float> in,int);
		void waitAndreceiveData(std::vector<float>* in,int);
		void waitAndreceiveData2(std::vector<float>* out, int n);
};
