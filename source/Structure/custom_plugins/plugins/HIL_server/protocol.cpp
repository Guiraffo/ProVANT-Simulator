#include <iostream>
#include "protocol.hpp"

Frame receive(Serial& serial)
{
  Frame frame;
  u8 header = frame.header();
  u8 end = frame.end();
  i32 status = 0;
  u8 last = 'a';
  bool fim = false;
  while (!fim)
  {
    // std::cout << "entrei" << std::endl;
    std::string byte = serial.read();
    u8 b[1];
    b[0] = byte[0];

    switch (status)
    {
      case 1:
        if (b[0] == last && last == header && header == end)
          break;
        if (b[0] != end)
          frame.addByte(b[0]);
        if (b[0] == end)
        {
          frame.addEnd();
          status = 0;
          // std::cout << "Frame received\n\n";
          fim = true;
        }
        break;
      case 0:
        if (b[0] == header)
        {
          frame.addHeader();
          status = 1;
        }
        break;
    }
    last = b[0];
  }
  return frame;
}

void print_frame(Frame frame, u8 type)
{
  // std::cout << "Raw frame:" << std::endl;
  // u8 *data = frame.data();
  // i32 size  = frame.size();
  // print_buffer(data,size);
  // std::cout << "\n\n";
  if (frame.unbuild())
  {
    u8* data = frame.data();
    i32 size = frame.size();
    std::cout << "Decoded frame:" << std::endl;
    if (type == FLOAT_FRAME)
      print_float_buffer(frame);
    else
      print_buffer(frame.data(), frame.size());

    frame.build();
    std::cout << "\n\n\n";
  }
  else
  {
    std::cout << "Error: corrupted frame...\n";
  }
}

void print_buffer(u8* data, i32 size)
{
  for (int i = 0; i < size; i++)
    std::cout << data[i];
}

void print_float_buffer(Frame frame)
{
  i32 num_floats = frame.size() / sizeof(float);
  for (int i = 0; i < num_floats; i++)
  {
    float n = frame.getFloat();
    std::cout << n << std::endl;
  }
}

void delay(unsigned long ms)
{
  usleep(ms * 1000);
}
