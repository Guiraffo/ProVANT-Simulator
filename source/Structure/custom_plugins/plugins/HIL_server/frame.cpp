#include "frame.hpp"
#include <cassert>

Frame::Frame() : _header(0x7E), _end(0x7E), _escape(0x7D), _cksum(0), _complete(false), _status(false)
{
  _buffer = NULL;
  _buffer_alloc_size = 0;
  _buffer_size = 0;
  _data = NULL;
  _data_alloc_size = 0;
  _data_size = 0;
  _current = 0;
}

u8 Frame::header()
{
  return this->_header;
}
u8 Frame::end()
{
  return this->_end;
}
u8 Frame::escape()
{
  return this->_escape;
}
i32 Frame::size()
{
  return this->_data_size;
}
u8* Frame::buffer()
{
  return this->_buffer;
}
i32 Frame::buffer_size()
{
  return this->_buffer_size;
}
u8* Frame::data()
{
  return this->_data;
}

/*  changes current data buffer. DELETES old data. */
void Frame::setData(u8* data, i32 size, i32 alloc_size, bool delOldData)
{
  assert(data != NULL);
  assert(size > 0);

  /* to ensure that the _Data buffer is dynamic allocated */
  if (alloc_size <= 0)
    _data_alloc_size = size;
  else
    _data_alloc_size = alloc_size;
  u8* aux = new u8[_data_alloc_size];
  memcpy(aux, data, size);
  /* end */

  if (delOldData)
    delete[] _data;

  _data = aux;
  _data_size = size;
}
void Frame::setHeader(u8 header)
{
  this->_header = header;
}
void Frame::setEnd(u8 end)
{
  this->_end = end;
}
void Frame::setEscape(u8 escape)
{
  this->_escape = escape;
}
void Frame::setSize(i32 size)
{
  this->_buffer_size = size;
}

void Frame::setDataSize(i32 size)
{
  assert(size > 0);
  u8* aux = (u8*)realloc((void*)_data, size);
  _data = aux;
  _data_alloc_size = size;
  assert(_data != NULL);
}

// void* Frame::realoc(void** ptr, size_t old_size, size_t new_size)
// {
// 	size_t n;
// 	void *aux = malloc(new_size);
// 	if (old_size > new_size) {
// 		n = new_size;
// 	} else {
// 		n = old_size;
// 	}
// 	if (*ptr != NULL)
// 		memcpy(aux,*ptr,n);
// 	free(*ptr);
// 	return aux;
// }

/* change the size allocated to the buffer */
void Frame::setBufferSize(i32 size)
{
  assert(size > 0);
  assert(size > 0);
  _buffer = (u8*)realloc((void*)_buffer, size);
  _buffer_alloc_size = size;
  assert(_buffer != NULL);
}

/* fletcher-16 checksum implementation from wikipedia */
u16 Frame::checksum(u8* data, int count)
{
  assert(count > 0);
  assert(data != NULL);
  u16 sum1 = 0;
  u16 sum2 = 0;
  i32 index;

  for (index = 0; index < count; ++index)
  {
    sum1 = (sum1 + data[index]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return (sum2 << 8) | sum1;
}

void Frame::addByte(u8 byte)
{
  addBytes2Buffer(&byte, 1);
}

void Frame::addHeader()
{
  addBytes2Buffer(&_header, 1);
}

void Frame::addEnd()
{
  addBytes2Buffer(&_end, 1);
  _complete = true;
}

void Frame::addBytes(u8* bytes, i32 size)
{
  assert(size > 0);
  assert(bytes != NULL);
  addBytes2Buffer(bytes, size);
}

void Frame::addBytes2Data(u8* bytes, i32 size)
{
  assert(size > 0);
  assert(bytes != NULL);
  if (_data_size + size > _data_alloc_size)
    setDataSize(_data_size + size + 10);

  memcpy(_data + _data_size, bytes, size);
  _data_size += size;
}
void Frame::addFloat(float num)
{
  u8 size = sizeof(float);
  u8* buf = new u8[size];
  memcpy(buf, &num, size);
  addBytes2Data(buf, size);
  delete[] buf;
}

float Frame::getFloat()
{
  size_t size = sizeof(float);
  float num;
  assert(_current + size <= _data_size);
  memcpy(&num, _data + _current, size);
  _current += size;
  return num;
}

void Frame::addDouble(double num)
{
  u8 size = sizeof(double);
  u8* buf = new u8[size];
  memcpy(buf, &num, size);
  addBytes2Data(buf, size);
  delete[] buf;
}
double Frame::getDouble()
{
  u8 size = sizeof(double);
  double num;
  assert(_current + size > _data_size);
  memcpy(&num, _data + _current, size);
  _current += size;
  return num;
}

void Frame::addInt(i32 num)
{
  u8 size = sizeof(i32);
  u8* buf = new u8[size];
  memcpy(buf, &num, size);
  addBytes2Data(buf, size);
  delete[] buf;
}

i32 Frame::getInt()
{
  u8 size = sizeof(i32);
  i32 num;
  assert(_current + size > _data_size);
  memcpy(&num, _data + _current, size);
  _current += size;
  return num;
}

/* add data buffer bytes as is to the buffer */
void Frame::addData2Buffer()
{
  addBytes2Buffer(_data, _data_size);
}

/* add bytes to buffer without adding escape flags */
void Frame::addBytes2Buffer(u8* bytes, i32 size)
{
  assert(size > 0);
  assert(bytes != NULL);
  if (_buffer_size + size > _buffer_alloc_size)
    setBufferSize(_buffer_size + size);

  memcpy(_buffer + _buffer_size, bytes, size);
  _buffer_size += size;
}

/* calculate and add the checksum to the 'data' buffer */
void Frame::addChecksum()
{
  /* header and end of frame are not considered in checksum */
  _cksum = checksum(_data, _data_size);
  u8* buf = new u8[2];
  memcpy(buf, &_cksum, 2);
  addBytes2Data(buf, 2);
  delete[] buf;
}

/* contruct the frame */
void Frame::build()
{
  assert(_data_size > 0);
  assert(_data != NULL);
  addChecksum();
  insertEscape();
  /* build frame on the 'buffer' buffer */
  addHeader();
  addData2Buffer();
  addEnd();
  _status = true;
}

/* contruct the frame with the provided data */
void Frame::build(u8* data, i32 size)
{
  assert(size > 0);
  assert(data != NULL);
  /* to secure in case of static array passed */
  setData(data, size);
  build();
}

/* Calculates the frame checksum and check if matches the checksum in the
 * frame the buffer must be complete, a read with status = 1 is a requirement
 * to this function. This function assumes you already removed escape bytes
 * and checksum from the data(_data array).
 */
bool Frame::check()
{
  return _status = _cksum == checksum(_data, _data_size);
}

i32 Frame::insertEscape()
{
  _data_size = insertEscape(&_data, _data_size);
}

/* check and insert escapes on data if any bytes matches header or end */
i32 Frame::insertEscape(u8** data, i32 size)
{
  std::vector<u8> buf;
  assert(size > 0);
  assert(data != NULL);

  for (i32 i = 0; i < size; i++)
  {
    u8 c = (*data)[i];
    if (isEscapable(c))
    {
      buf.push_back(_escape);
      buf.push_back(c ^ 0x20);
    }
    else
    {
      buf.push_back(c);
    }
  }

  // data = new u8[buf.size()];
  *data = (u8*)realloc((*data), buf.size() * sizeof(u8));
  for (i32 i = 0; i < buf.size(); i++)
    (*data)[i] = buf[i];
  assert(buf.size() > 0);
  assert(buf.size() >= size);
  assert(*data != NULL);

  return buf.size();
}

bool Frame::isEscapable(u8 byte)
{
  return (byte == _header || byte == _end || byte == _escape);
}

i32 Frame::removeEscape()
{
  _data_size = removeEscape(&_data, _data_size);
}

/* check and insert escapes on data if any bytes matches header or end */
i32 Frame::removeEscape(u8** data, i32 size)
{
  assert(size > 0);
  assert(*data != NULL);
  std::vector<u8> buf;

  for (i32 i = 0; i < size - 1; i++)
  {
    if ((*data)[i] == _escape)
    {
      i++;  // jump the escape byte
      buf.push_back((*data)[i] ^ 0x20);
    }
    else
    {
      buf.push_back((*data)[i]);
    }
  }
  /* The for above checked until n-1, the last one is included in
   * case n-2 was not an escape
   */
  if ((*data)[size - 2] != _escape)
    buf.push_back((*data)[size - 1]);

  *data = (u8*)realloc((*data), buf.size() * sizeof(u8));
  for (i32 i = 0; i < buf.size(); i++)
    (*data)[i] = buf[i];

  assert(buf.size() > 0);
  assert(*data != NULL);

  return buf.size();
}

bool Frame::unbuild()
{
  copyBuffer2data();
  removeEscape();
  memcpy(&_cksum, _data + _data_size - 2, 2);
  _data_size -= 2;
  _buffer_size = 0;

  return check();
}

/* Copy data from buffer with checksum. checksum might have escape bytes, so
 * is necessary to remove escape bytes and then remove checksum to _cksum.
 */
void Frame::copyBuffer2data()
{
  assert(_buffer_size > 2);
  _data_size = _buffer_size - 2;
  setDataSize(_data_size);
  memcpy(_data, _buffer + 1, _data_size);
}

/* delete current frame and restarts the frame */
void Frame::clear()
{
  _status = false;
  _complete = false;
  _buffer_size = 0;
  _data_size = 0;
  _cksum = 0;
  _current = 0;
  int i;
  if (_data != NULL)
    for (i = 0; i < _data_alloc_size; i++)
      _data[i] = 0;
  if (_buffer != NULL)
    for (i = 0; i < _buffer_alloc_size; i++)
      _buffer[i] = 0;
}

Frame::~Frame()
{
  // delete[] _buffer;
  // delete[] _data;
}
