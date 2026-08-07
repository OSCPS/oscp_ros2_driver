#ifndef IMUREAD_H_
#define IMUREAD_H_

#include <stdint.h>

#define MAGBUFFSIZE 128


typedef struct
{
    float x;
    float y;
    float z;

} Point_t;



typedef struct
{
    float V[3];                  // hard iron offset
    float invW[3][3];            // inverse soft iron matrix

    float B;                     // magnetic field magnitude
    float FourBsq;

    float FitError;
    float FitErrorAge;


    float trV[3];
    float trinvW[3][3];

    float trB;
    float trFitErrorpc;


    float A[3][3];
    float invA[3][3];


    float matA[10][10];
    float matB[10][10];


    float vecA[10];
    float vecB[4];


    int8_t ValidMagCal;


    int16_t BpFast[3][MAGBUFFSIZE];

    int8_t valid[MAGBUFFSIZE];

    int16_t MagBufferCount;


} MagCalibration_t;



extern MagCalibration_t magcal;


#endif