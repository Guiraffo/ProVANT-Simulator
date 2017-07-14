// https://en.wikibooks.org/wiki/Serial_Programming/termios

#include "serial.h"
#include <unistd.h>
#include <stdlib.h>

int open_port(const char* in)
{
	int fd; 

	//fd = open("/dev/ttyS0", O_RDWR /*| O_NOCTTY  | O_NDELAY*/);
	fd = open(in, O_RDWR |O_NOCTTY/*  | O_NDELAY*/);
	if (fd == -1){
		perror("open_port: Unable to open");
		printf("falha");
		exit(0);
	}
	
	struct termios options;
	tcgetattr(fd, &options);
	// velocidade	
	cfsetispeed(&options, B1152000);
	cfsetospeed(&options, B1152000);
		
	options.c_cflag |= (CLOCAL | CREAD);
	// sem paridade 8n1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |=CS8;
	// raw input
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	
 	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
	options.c_oflag = 0;

	tcsetattr(fd, TCSANOW, &options);

	return (fd);
}

void write_hex_port(int fd,int value)
{
	printf("write_hex_port\n");
	union u variable;
	variable.i = value;
	printf("%x\n",variable.i);
	for (int i = 3; i>=0 ; i--)
	{ 
		char buff[1];
		buff[0] = variable.s[i] & 0x000000FF;
		printf("%02x\n",buff[0]);
		int n = write(fd, buff, 1);
		if (n < 0) fputs("write() of 1 byte failed!\n", stderr);
		if(n == 0) printf("byte não enviado");		
		/*while(1)
		{
			//printf("loop\n");
			int ok = (int)read(fd, buff, sizeof (buff));
			if(ok>0) break;
		}*/ // ack
		//printf("recebi\n");			
	}
}


void write_float_port(int fd,float value)
{
	int i=0;
	int count=0;
	union u variable;
	variable.i = 0;
	variable.f = value;
	for (i = 3; i>=0 ; i--)
	{ 
		char buff[1];
		memset(buff, 0, sizeof buff);
		buff[0] = variable.s[i];	
		int n = write(fd, buff, sizeof (buff));
		//printf("Enviado %02x   ",buff[0]);	
		if (n < 0) fputs("write() of 1 byte failed!\n", stderr);
	}
		while(1)
		{
			char buff2[1];
			int ok = 0;
			ok = (int)read(fd, buff2, sizeof (buff2));
			if(ok>0)
			{
				//printf("Recebido %02x   \n",buff2[0]);
				break;
			}
		}	
}

union u read_port(int fd)
{
	unsigned char buffer[1];
	union u variable;
	size_t offset = 3;
	size_t bytes_read;
	while (((int)offset) >= 0) {
		bytes_read = read (fd, buffer, sizeof (buffer));
		if(bytes_read>0)		
		{	
			//printf("byte %x",buffer[0]);
			variable.s[offset] = (buffer[0]);
			offset -= bytes_read;
		}	
	}
	return variable;
}

int read_startflag(int fd)
{
	int i = 0;
	int flag = 0;
	unsigned char buffer[1];
	union u variable;
	size_t bytes_read;
	while (1){
		bytes_read = read (fd, buffer, sizeof (buffer));
		if(bytes_read>0)		
		{	
			printf("buffer %x\n",buffer[0]);
			if((buffer[0]) == 0xff )
			{
				if(flag = 0)
				{	
					flag = 1; 
					i = 1;
				}
				else
				{
					i++;
					if(i>=4) return 1;
				}
			}
			else flag = 0;
		}	
	}
}

void close_port(int fd)
{
	close(fd);
}

