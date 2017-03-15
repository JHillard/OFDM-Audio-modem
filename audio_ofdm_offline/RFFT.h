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

    //Performns an in-place real FFT on input array x. Requires nx, number of real values and pointer x.
    //Performs the real fft by interleaving imaginary zeros into the array, and performing the hardware accelerated complex fft cfft().
    void rfft_noscale(int *x, ushort nx) {

     int complex_x[MAX_BUFFER_SIZE*2];
     for(int i=0; i<nx; i++){ //Interleaves real and complex numbers.
      complex_x[i*2] = x[i];
      complex_x[i*2+1] = 0;
     }

       cfft( (DATA *)complex_x, nx, NOSCALE);
       cbrev( (DATA *)complex_x, (DATA *)complex_x, nx);

     for(int i=0; i<nx; i++){ //Captures first half of signal
      x[i] = complex_x[i];
     }
    }


}

#endif /* RFFT_h */
