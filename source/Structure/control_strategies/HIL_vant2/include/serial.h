#if defined (__cplusplus)
extern "C"
{
#endif

	#include <stdio.h>   
	#include <string.h>  
	#include <unistd.h>  
	#include <fcntl.h>   
	#include <errno.h>   
	#include <termios.h> 
	#include <stdlib.h>

	union u
	{
	    unsigned int i;
	    float f;
	    char s[4];
	};

	int open_port(const char*);
	void write_hex_port(int fd,int value);
	void write_float_port(int fd,float value);
	union u read_port(int fd);
	int read_startflag(int fd);
	void close_port(int fd);

#if defined (__cplusplus)
}
#endif
