#ifndef MATRIX_H_
#define MATRIX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


void f3x3matrixAeqI(float A[][3]);

void fmatrixAeqI(float *A[], int16_t rc);

void f3x3matrixAeqScalar(float A[][3], float Scalar);

void f3x3matrixAeqAxScalar(float A[][3], float Scalar);

void f3x3matrixAeqMinusA(float A[][3]);

void f3x3matrixAeqInvSymB(float A[][3], float B[][3]);

float f3x3matrixDetA(float A[][3]);

void eigencompute(
    float A[][10],
    float eigval[],
    float eigvec[][10],
    int8_t n
);

void fmatrixAeqInvA(
    float *A[],
    int8_t iColInd[],
    int8_t iRowInd[],
    int8_t iPivot[],
    int8_t isize
);

void fmatrixAeqRenormRotA(float A[][3]);


#ifdef __cplusplus
}
#endif

#endif