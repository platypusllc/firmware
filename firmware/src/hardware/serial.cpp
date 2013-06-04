#include "serial.h"

Serial::Serial(bool isDefault)
{
  // If specified, set as default IO stream                                
  if (isDefault) {
    stdout = _stream;
    stdin = _stream;
  }
}

Serial::~Serial() { }

FILE * Serial::stream()
{
  return _stream;
}

