/** \file FEQ.h
   Designs, implements, and contains supporting methods for the FEQ.
*/
//
//  FEQ.h
//
//
//  Created by Terry Kong on 2/5/16.
//
//

#ifndef FEQ_h
#define FEQ_h

// Helpful Definitions
#define FIXED_FBITS       15
#define WORD_LENGTH       16
#define MAX_BUFFER_SIZE 2048

namespace FEQ {

    static long numerator[MAX_BUFFER_SIZE] = {0};
    static int denominator[MAX_BUFFER_SIZE] = {0};

    /** \brief
     Use long long to multiply to simulate MAC behavior
    */
    static long long FIXED_MUL(int a, int b) {
        return ((long long)a*(long long)b);
    }
    /** \brief
    Use long long and simulate round-off. Typically used after the MAC operation simulated by FIXED_MUL.
     */
    static int FIXED_RND(long long a) {
        return ((a + (long long) (1 << (FIXED_FBITS - 1))) >> FIXED_FBITS);
    }

  /** \brief
   in-place complex multiplication (result in b)
   length must be divisible by 2
*/
    void cmul(const int* a, int* b, int length) {
        int temp;
        for (int i = 0; i < length; i+=2) {
            temp = a[i]*b[i] - a[i+1]*b[i+1];
            b[i+1] = a[i]*b[i+1]+a[i+1]*b[i];
            b[i] = temp;
        }
    }
  /** \brief
     in-place complex multiplication (result in c)
    length must be divisible by 2
     c = (a.*b) * c
    */
    void cmulq15(const int* a, const int* b, int *c, int length) {
        long long real,imag;
        int t[MAX_BUFFER_SIZE];
        for( int i=0; i<length; i++){
         t[i] = FIXED_RND( FIXED_MUL(a[i], b[i]) );
        }
        cmul(t, c, length);
    };

  /** \brief
     x is the fft of the ref symbol.

     y is the fft of the ref symbol passed thru channel.

     rFrac is the frac part of the feq filter.

     rExp is the exp part of the feq filter.

          - the filter elements are feq[i] = rFrac[i]*rExp[i].
*/
    void getFEQ(int *x, int *y, int *rFrac, int *rExp, int length) {
        int denominatorLength = length/2;
        int c,d;
        for (int j = 0; j < denominatorLength; j++) {
            c = y[2*j];
            d = y[2*j+1];
            denominator[2*j] = FIXED_RND(FIXED_MUL(c,c)+FIXED_MUL(d,d));
            denominator[2*j+1] = denominator[2*j];
        }
        for (int i = 0; i < denominatorLength; i++) {
            numerator[2*i] = ((long)FIXED_RND(FIXED_MUL(x[2*i],y[2*i]) + FIXED_MUL(x[2*i+1],y[2*i+1])))<<WORD_LENGTH;
            numerator[2*i+1] = ((long)FIXED_RND(FIXED_MUL(x[2*i+1],y[2*i]) - FIXED_MUL(x[2*i],y[2*i+1])))<<WORD_LENGTH;
        }
        ldiv16((LDATA*)numerator,(DATA*)denominator,(DATA*)rFrac,(DATA*)rExp,length);
    }

    /** \brief
     In-place application of the FEQ.
     Requires a pointer to the symbol, the FEQ pointers rFrac and rExp, and int length.
     Modifies the passed symbol.
     */
    void applyFEQ(int *symbol, int *rFrac, int *rExp, int length) {

        cmulq15(rFrac,rExp,symbol,length);
    }
}

#endif /* FEQ_h */
