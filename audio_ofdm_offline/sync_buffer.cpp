#include "sync_buffer.h"

SyncBuffer::SyncBuffer()
{
    _buffer = NULL;
    _numElem = 0;
    _alignIndex = 0;
}

SyncBuffer::~SyncBuffer()
{
    delete[] _buffer;
}

void SyncBuffer::setup(int numElem)
{
    delete[] _buffer;
    _numElem = numElem;
    _buffer = new int[3 * _numElem];
    reset();
}

void SyncBuffer::reset()
{
    _alignIndex = _numElem;
    for(int n = 0; n < 3 * _numElem; n++)
    {
       _buffer[n] = 0; 
    }
}

void SyncBuffer::insert(const int *data)
{
    for(int n = 0; n < _numElem; n++)
    {
        _buffer[n] = _buffer[n + _numElem];
        _buffer[n + _numElem] = data[n];
    }
}

void SyncBuffer::setAlignIndex(int index)
{
    _alignIndex = index;
}

const int* SyncBuffer::getAlignedBuffer()
{
  return _buffer + _alignIndex;
}

const int* SyncBuffer::getFullBuffer()
{
  return _buffer;
}

int SyncBuffer::getAlignIndex()
{
   return _alignIndex; 
}

int SyncBuffer::getNumElem()
{
   return _numElem; 
}


