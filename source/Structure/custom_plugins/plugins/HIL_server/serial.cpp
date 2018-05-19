#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "serial.hpp"

	//Class constructor
Serial::Serial():port(io),quitFlag(false){}
	//Class destructor
Serial::~Serial()
{
	//Stop the I/O services
	io.stop();
	//Wait for the thread to finish
	runner.join();
}

const char* Serial::buffer()
{
	const char* bufPtr=boost::asio::buffer_cast<const char*>(_buffer.data());
	return bufPtr;
}

std::string Serial::read(int bytes)
{
	boost::asio::streambuf buffer;
	boost::system::error_code ec;
	// std::cout << "antes de ler" << std::endl;
	std::size_t n =  0;
	char c;
	std::string s;
	while (n < bytes) {
		n += boost::asio::read(port, boost::asio::buffer(&c,1),boost::asio::transfer_exactly(1), ec);
		s += c;

		// std::cout << n << std::endl;
	}
	// std::cout << n << std::endl;
	// std::cout << "depois ler" << std::endl;
	
	// std::cout << "depois" << std::endl;	
	// if (ec)
	// {
	// 	std::cout << "erro" << std::endl;
	// 	// An error occurred.
	// }
	// else
	// {
	// 	std::cout << n << std::endl;
	// 	// n == 128
	// }
	// return buffer;
	// char* bufPtr= (char*)boost::asio::buffer_cast<const char*>(buffer.data());
	// char* bufPtr= (char*)buffer.consume(bytes);
	// std::cout << "antes de retornar read" << std::endl;
	return s;
}	


//Connection method that will setup the serial port
bool Serial::connect(const std::string& port_name, int baud)
{
	try
	{
		using namespace boost::asio;
		port.open(port_name);
		//Setup port
		port.set_option(serial_port::baud_rate(baud));
		port.set_option(serial_port::flow_control(serial_port::flow_control::none));
		std::cout << "port: " << port_name << "; baud rate: " << baud << "\n";
		/*if (port.is_open())
		{
			//Start io-service in a background thread.
			//boost::bind binds the ioservice instance
			//with the method call
			runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
			//startReceive();
		}*/
	} 
	catch(boost::system::system_error& e)
	{
		std::cout << "Error opening serial port " << port_name << " at baud rate " << baud << std::endl; 
		// std::cout<<"Error: " << e.what()<<std::endl;
		// std::cout<<"Info: "  << boost::diagnostic_information(e) <<std::endl;
		return false;
	}

	return port.is_open();
}

//The method that will be called to issue a new 
//asynchronous read
void Serial::startReceive()
{
	// using namespace boost::asio;
	//Issue a async receive and give it a callback
	//onData that should be called when "\r\n"
	//is matched.
	//async_read_until(port, buffer,"\r\n",functor);
	boost::asio::async_read_until(port, _buffer,"\r\n",boost::bind(&Serial::onData,this, _1,_2));
	//  boost::asio::async_read(port, buffer,boost::asio::transfer_at_least(1),boost::bind(&Serial::onData,this, _1,_2));
} 

void Serial::onData(const boost::system::error_code& e,std::size_t size)
{
	if (!e)
	{
		std::istream is(&_buffer);
		std::string data(size,'\0');
		is.read(&data[0],size);
		std::cout<<"Received data:"<<data;
		//If we receive quit()\r\n indicate
		//end of operations
		quitFlag = (data.compare("quit()\r\n") == 0);
	}

	startReceive();
}
	  
//Function for sending a data string
void Serial::send(/*const*/ void * text, size_t size)
{
	char* aux = (char*)text;
	//boost::asio::write(port, boost::asio::buffer(text,size));
	for(int i=0;i<size;i++)
	{
		boost::asio::write(port, 
		boost::asio::buffer(aux+i,1));	
		// estourando FIFO!!!!!		
		usleep( 20 );
	}
	/*int i = 0;
	int j = 0;
	while(true)
	{
		boost::asio::write(port, 
		boost::asio::buffer(aux+i,1));
		i++;
		j++;
		if(j>=8)
		{	
			usleep( 200 );
			j = 0;
		}
		if(i>=size) break;		
	}*/
}
	  
//Quit Function
bool Serial::quit(){return quitFlag;}

