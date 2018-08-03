#ifndef FRAME_H
#define FRAME_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>

typedef	uint8_t	u8;
typedef	int8_t	i8;
typedef	uint16_t u16;
typedef	int16_t i16;
typedef int32_t	i32;
typedef uint32_t	u32;

class Frame {
private:
	u8 _header;
	u8 _end;
	u8 _escape;
	u8 *_data; /* frame message data */
	i32 _data_size; /* size of _data */
	i32 _data_alloc_size; /* alloced size of buffer */
	u8 *_buffer; /* final buffer to send the frame */
	i32 _buffer_size; /* size of buffer */
	i32 _buffer_alloc_size; /* alloced size of buffer */
	u16 _cksum; /* calculated checksum */
	i32 _current;
	bool _status; /* package verified and checksum ok? */
	bool _complete; /* is package done? has _end already been added? */

public:
	Frame();

	u8 header();
	u8 end();
	u8 escape();
	i32 size();
	u8 *buffer();
	i32 buffer_size();
	u8 *data();

	void setData(u8 *data, i32 size, i32 alloc_size = -1,
			bool delData = true);

	void setHeader(u8 header);
	void setEnd(u8 end);
	void setEscape(u8 escape);
	void setSize(i32 size);
	void setBufferSize(i32 size);
	void setDataSize(i32 size);
	void* realoc(void** ptr, size_t old_size, size_t new_size);
	void* realoc(void* ptr, size_t old_size, size_t new_size);
	u16 checksum(u8 *data, int count);

	/* add* functions add data to buffer */
	void addEnd();
	void addHeader();
	void addByte(u8 byte);
	void addBytes(u8 *bytes, i32 size);
	void addFloat(float n); 
	float getFloat();
	void addDouble(double n); 
	double getDouble();
	void addInt(i32 n); 
	i32 getInt();
	void addData2Buffer();
	void addBytes2Buffer(u8 *bytes, i32 size);
	void addBytes2Data(u8 *bytes, i32 size);
	bool isEscapable(u8 byte);

	/* calculates and escapes the checksum */
	void addChecksum();
	bool check();
	void build(u8 *data, i32 size);
	void build();
	bool unbuild();
	i32 insertEscape();
	i32 insertEscape(u8** data, i32 size);
	i32 removeEscape();
	i32 removeEscape(u8** data, i32 size);
	void copyBuffer2data();
	void clear();

	~Frame();
};
#endif
