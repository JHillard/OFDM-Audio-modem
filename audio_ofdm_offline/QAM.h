//
//  QAM.h
//
//
//  Created by Terry Kong on 1/30/16.
//
//

#ifndef QAM_H
#define QAM_H

namespace QAM {

     static const int scaleShiftFac = 12;
     static const int centerShift = 8;
     static const int tableSize = 17;
     static const int qamTable[17][17] = {
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3},
     {1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3}
     };


    /*
    static const int scaleShiftFac = 11;
    static const int centerShift = 16;
    static const int tableSize = 33;
    static const int qamTable[33][33] = {
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,4,4,8,8,8,8,8,8,8,8,12,12,12,12,12,12,12,12},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {1,1,1,1,1,1,1,1,1,5,5,5,5,5,5,5,5,9,9,9,9,9,9,9,9,13,13,13,13,13,13,13,13},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {2,2,2,2,2,2,2,2,2,6,6,6,6,6,6,6,6,10,10,10,10,10,10,10,10,14,14,14,14,14,14,14,14},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15},
        {3,3,3,3,3,3,3,3,3,7,7,7,7,7,7,7,7,11,11,11,11,11,11,11,11,15,15,15,15,15,15,15,15}
    };*/
    //Demodulates a recieved input and stores it in output. Requires pointers to input and output, along with the inputLength.
    //Demodulation performed on the extracted data carriers. By default the QAM constellation is of symbol size four, and can be found in the qamTable variable.
    void demod(const int *input, int* output, int inputLength) {
        int real_col, imag_row;
        int j = 0;
        for (int i = 0; i < inputLength; i+=2) {
            real_col = input[i] >> scaleShiftFac;
            imag_row = input[i+1] >> scaleShiftFac;
            // shift center
            real_col = centerShift + real_col;
            imag_row = centerShift - imag_row;
            output[j++] = qamTable[imag_row][real_col];
        }
    };

};

#endif /* QAMdemod_H */
