#ifndef SERIAL_H
#define SERIAL_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>

class Serial
{
private:
  // Pointer of the callback function that will be executed when data
  // arrives.
  boost::function<void(const boost::system::error_code& e, std::size_t size)> functor;

  // Buffer in which to read the serial data
  boost::asio::streambuf _buffer;
  // quit Flag
  bool quitFlag;
  // vetor de dados de entrada
  std::vector<double> input;

  // Boost.Asio I/O service required for asynchronous
  // operation
  boost::asio::io_service io;
  // private:
  // Serial port accessor class
  boost::asio::serial_port port;
  // Background thread
  boost::thread runner;

public:
  // Class constructor
  Serial();
  // Class destructor
  ~Serial();

  const char* buffer();
  std::string read(int bytes = 1);
  // Connection method that will setup the serial port
  bool connect(const std::string& port_name, int baud = 9600);

  // Function for sending a data string
  void send(/*const*/ void* text, size_t size);

  // Quit Function
  bool quit();
};

#endif
