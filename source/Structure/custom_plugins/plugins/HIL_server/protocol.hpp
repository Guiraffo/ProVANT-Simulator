#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "frame.hpp"
#include "serial.hpp"

#define FLOAT_FRAME 1
#define STR_FRAME 0

void print_frame(Frame frame, u8 type);
void print_buffer(u8* data, i32 size);
void print_float_buffer(Frame frame);
Frame receive(Serial& serial);
void delay(unsigned long ms);

#endif
