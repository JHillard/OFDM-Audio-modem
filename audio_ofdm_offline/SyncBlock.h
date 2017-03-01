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
    
    static int FIXED_RND(long long a) {
        return ((a + (long long) (1 << (FIXED_FBITS - 1))) >> FIXED_FBITS);
    }
 
    int mean(const int* data, int length) {
        long long result = 0;
        for (int i = 0; i < length; i++) {
            result += data[i];
        }
        result /= length;
        return result;
    }
    
    void subtractMean(int* data, int length) {
        int m = mean(data,length);
        for (int i = 0; i < length; i++) {
            data[i] -= m;
        }
    }
    
    // dataLength should be even
    ushort xcorr(const int* data, int* syncSymbol, int* output, int outputLen, int syncSymLen) {
        
        // Your code here
        
        return 0;
    };

    // Linear search for maximum
    void absMax(const int* array, int length, int &maxValue, int &maxIndex) {
        
        // Your code here
        // If length is less than 1, do nothing (return) 
    };
    
    // Linear search for maximum
    void absMax(const int* array, int length, int &maxValue) {
        int maxIndex;
        absMax(array,length,maxValue, maxIndex);
    };
    
    void xcorrMax(int* data, int* syncSymbol, int* output, int outputLen, int syncSymLen,int dataLength, int &maxValue, int &maxIndex) {
        xcorr(data,syncSymbol,output,outputLen,syncSymLen);
        absMax(output,outputLen,maxValue,maxIndex);
    };
    
    int calculateEnergy(const int* a, int length) {
        
        // Your code here
        
        return 0;
    }
};

#endif /* SyncBlock_H */
