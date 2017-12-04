#include <boost/asio.hpp>
#include <boost/thread.hpp>


class SerialClass{
		public:
		  //Class constructor
		  SerialClass():port(io),quitFlag(false){}
		  //Class destructor
		  ~SerialClass()
		  {
		  		{
					//Stop the I/O services
					io.stop();
					//Wait for the thread to finish
					runner.join();
				}
		  
		  }
		  //Connection method that will setup the serial port
		  bool connect(const std::string& port_name, int baud = 9600)
		  {
		  		try
				{
					using namespace boost::asio;
					port.open(port_name);
					//Setup port
					port.set_option(serial_port::baud_rate(baud));
					port.set_option(serial_port::flow_control(serial_port::flow_control::none));

					if (port.is_open())
					{
						//Start io-service in a background thread.
						//boost::bind binds the ioservice instance
						//with the method call
						runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
						startReceive();
					}
	
				} 
				catch(boost::system::system_error& e)
				{
					std::cout<<"Error: " << e.what()<<std::endl;
					std::cout<<"Info: "  << boost::diagnostic_information(e) <<std::endl;
					return 1;
				}

				return port.is_open();
		  }
		  //The method that will be called to issue a new 
		  //asynchronous read
		  void startReceive()
		  {
		  		using namespace boost::asio;
				//Issue a async receive and give it a callback
				//onData that should be called when "\r\n"
				//is matched.
				//async_read_until(port, buffer,"\r\n",functor);
				//async_read_until(port, buffer,"\r\n",boost::bind(&SerialClass::onData,this, _1,_2));
				async_read(port, buffer,boost::asio::transfer_at_least(1),boost::bind(&SerialClass::onData,this, _1,_2));
				
		  
		  }
		  
		  void onData(const boost::system::error_code& e,std::size_t size)
		  {
				if (!e)
				{
					std::istream is(&buffer);
					std::string data(size,'\0');
					is.read(&data[0],size);

					std::cout<<"Received data:"<<data;

					//If we receive quit()\r\n indicate
					//end of operations
					quitFlag = (data.compare("quit()\r\n") == 0);
				};

				startReceive();
		  };
		  
		  //Function for sending a data string
		  void send(const std::string& text)
		  {
		  		boost::asio::write(port, boost::asio::buffer(text));
		  }
		  
		  //function for reading
		  /*void read()
		  {
		 		boost::asio::read(port,boost::asio::buffer(input),boost::asio::transfer_exactly(48));
		  }*/
		  
		  //Quit Function
		  bool quit(){return quitFlag;}
		  // Pointer of the callback function that will be executed when data 
		  //arrives.
		  boost::function<void(const boost::system::error_code& e,std::size_t size)> functor;
		  
		  //Buffer in which to read the serial data
		  boost::asio::streambuf buffer;
		  // quit Flag
		  bool quitFlag;
		  // vetor de dados de entrada
		  std::vector<double> input;
  		private:
		  //Boost.Asio I/O service required for asynchronous 
		  //operation
		  boost::asio::io_service io;
		  //Serial port accessor class
		  boost::asio::serial_port port;
		  //Background thread
		  boost::thread runner;
};
