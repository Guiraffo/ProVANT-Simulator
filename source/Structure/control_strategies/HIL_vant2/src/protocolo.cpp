#include "protocolo.hpp"
#include "iostream"

protocolo::protocolo(std::string in)
{
	std::cout << "protocolo" << std::endl;
	fd = open_port(in.c_str());
}

protocolo::~protocolo()
{
	std::cout << "~protocolo" << std::endl;
	close_port(fd);
}
void protocolo::sincronization()
{
	std::cout << "sincronization" << std::endl;
	int nstarts = 0;
	char buff[1];
	size_t bytes_read;	
	buff[0] = 0xFE;
	int n = write(fd, buff, sizeof (buff));
	while(1)
	{	
		unsigned char buffer = 0x00;
		bytes_read = read (fd, &buffer, sizeof (buffer));
		if(bytes_read >0)
		{	
			//printf("Recebido %02x  ",buffer);
			bytes_read = 0;
			if(buffer==0xfd)
			{
				nstarts++;
			}
			else
			{
				nstarts = 0;
			}	
			if(nstarts==4)
			{
				printf("\n");
				return;
			}
			buff[0] = 0x000000FE;
			int n = write(fd, buff, sizeof (buff));
			//printf("Enviado %02x  \n",buff[0]);
			bytes_read=0;
		}
	}
	
}

void protocolo::sendData(std::vector<float> in, int n)
{
	for(int i=0;i<n;i++)
	{
                //std::cout << "in: " << in.at(i) << std::endl;
		write_float_port(fd,in.at(i));
	}
}

void protocolo::waitAndreceiveData(std::vector<float>* out, int n)
{
	//std::cout << "waitAndreceiveData" << std::endl;	
	union u variable; 
	int i= 0;	

	while(true)
	{
		if(read_startflag(fd))
		{	
			while(i<n)
			{
				variable = read_port(fd);
				write(fd, &(variable.s), 1);
				out->push_back(variable.f);
                                //std::cout << "out: " << out->at(i) << std::endl;
				//std::cout << "i: " << i << std::endl;
				i++;
				//if(i>MAX_SIZE_DATA) std::cout << "MAX_SIZE_DATA" << std::endl;	; 
			}
			return;
		}
	}
}

void protocolo::waitAndreceiveData2(std::vector<float>* out, int n)
{
	int nfloats = 0;  
	int interator = 4;
	int nstarts = 0;

	union u number;	

	while(nfloats<n)
	{
		unsigned char buffer[1];
		size_t bytes_read;					
		bytes_read = read (fd, buffer, sizeof (buffer));
		if(bytes_read >0)
		{	
			//printf("Recebido %02x   ",buffer[0]);	
			interator--;
			number.s[interator] = (char)buffer[0];
			if(interator==0)
			{
				write(fd, &buffer, 1);	
				//printf("Enviado %02x   \n",buffer[0]);
				out->at(nfloats) = number.f;
                               	interator = 4;
				nfloats++;
				number.i = 0;				
			} 
		}
	}
}	




