//
//  SyncBlock.h
//
//
//  Created by Terry Kong on 1/30/16.
//
//

#ifndef SyncBlock_H
#define SyncBlock_H

#include "dsplib.h"
#include "stdlib.h"

// Helpful Definitions
#define FIXED_FBITS       15
#define WORD_LENGTH       16
#define MAX_BUFFER_SIZE 2048

#define MAX_INT32        2147483647u

namespace SyncBlock {

    // Use long long to multiply to simulate MAC behavior
    static long long FIXED_MUL(int a, int b) {
        return ((long long)a*(long long)b);
    }

    // Use long long and simulate round-off. Typically used after the MAC operation simulated by FIXED_MUL.
    static int FIXED_RND(long long a) {
        return ((a + (long long) (1 << (FIXED_FBITS - 1))) >> FIXED_FBITS);
    }

    //returns the mean of the input array data. Requires length of data and pointer to data as input.
    int mean(const int* data, int length) {
        long long result = 0;
        for (int i = 0; i < length; i++) {
            result += data[i];
        }
        result /= length;
        return result;
    }
    //Demeans an array of data. Requires length of data and pointer to data as input.
    void subtractMean(int* data, int length) {
        int m = mean(data,length);
        for (int i = 0; i < length; i++) {
            data[i] -= m;
        }
    }

    // dataLength should be even
    ushort xcorr(const int* data, int* syncSymbol, int* output, int outputLen, int syncSymLen) {
      return convol1((DATA *)data, (DATA *)syncSymbol, (DATA *)output, outputLen, syncSymLen);
        return 0;
    };

    // Linear search for maximum
    void absMax(const int* array, int length, int &maxValue, int &maxIndex) {
       // If length is less than 1, do nothing (return)
      if(length<1) return;

      int mx = array[0];
      int mi = 0;
      for( int i=0; i<length; i++){
         int t =array[i];
         if ( t>mx){
           mx = t;
           mi = i;
         }
        }
        maxValue = (array[mi]);
        maxIndex = mi;
    };

    // Linear search for maximum
    void absMax(const int* array, int length, int &maxValue) {
        int maxIndex;
        absMax(array,length,maxValue, maxIndex);
    };
   //Takes the correlation of an input data and the sync symbol, and stores the max value and index of the correlation in maxValue and maxIndex.
   //Requires input data pointer, length of input pointer, the syncSymbol, length of the syncSymbol, as well as pointers to the output of the correlation. The addresses of maxValue and maxIndex are also required.
    void xcorrMax(int* data, int* syncSymbol, int* output, int outputLen, int syncSymLen,int dataLength, int &maxValue, int &maxIndex) {
        xcorr(data,syncSymbol,output,outputLen,syncSymLen);
        absMax(output,outputLen,maxValue,maxIndex);
    };

    //Calculates the energy of a signal stored in array a. Requires length.
    //Total energy calculated via the sum from 0 to length-1 of a[n]^2.
    int calculateEnergy(const int* a, int length) {

      //Energy = sum(n=0:Length, a[n]^2 )
       int energy = 0;
       for( int n=0; n<length; n++){
        energy = energy + FIXED_RND(FIXED_MUL(a[n],a[n]) );
       }
       return energy;
    }
};

#endif /* SyncBlock_H */
