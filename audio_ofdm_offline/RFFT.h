//
//  RFFT.h
//  
//
//  Created by Terry Kong on 2/12/16.
//
//

#ifndef RFFT_h
#define RFFT_h

// Helpful Definitions
#define MAX_BUFFER_SIZE 2048

#include "dsplib.h"

namespace RFFT {
    
    static int fftOutput[MAX_BUFFER_SIZE] = {0};

    void rfft_noscale(int *x, ushort nx) {
    
        // Your code here
    }
    
}

#endif /* RFFT_h */
