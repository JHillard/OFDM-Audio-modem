#ifndef sync_buffer_h
#define sync_buffer_h

/** \file

TBD

*/

#include "core.h"
#include "OLED.h"

/** \brief SyncBuffer
  
  Buffer object to handle unsynchronized received data blocks.
  Two data blocks are stored to allow correlation with sync symbol
  to determine frame alignment.  The implementation mimics that of
  a "shift register" where each "register" is a block of data.
*/
class SyncBuffer {
    
public:

    /** \brief Constructor
    \param numElem number of elements (int) per block
    */
    SyncBuffer();
  
    /** \brief Destructor
    \param blockSize number of elements (int) per block
    \param numBlocks number of blocks 
    */    
    ~SyncBuffer();
    
    /** \brief Setup
    \param numElem number of elements (int) per block
    */
    void setup(int numElem);
    
    /** \brief Reset
    Set buffer elements and alignIndex to zero
    */
    void reset();
    
    /** \brief insert 
    Shifts previous blocks of data and insert a new data block at the beginning
    of the buffer.  The oldest data block gets pushed out of the buffer.
    */    
    void insert(const int *data);

    /** \brief getFullBuffer 
    Pointer to start of buffer.
    */
    const int* getFullBuffer();
    
    /** \brief setAlignIndex 
    This is an index indicating the begining of an aligned block of data.
    Set to zero by the constructor.
    */
    void setAlignIndex(int index);
    
    /** \brief getAlignedBuffer 
    Pointer to start of aligned buffer.
    */
    const int* getAlignedBuffer();
    
    /** \brief getAlignIndex 
    Returns align index.
    */
    int getAlignIndex();
    
    /** \brief getNumElem 
    Returns buffer number of elements.
    */
    int getNumElem();
    
    
private:
    int *_buffer;    // Buffer[2 x numElem]
    int _numElem;    // Number of elements per block
    int _alignIndex; // Align index
};




#endif
