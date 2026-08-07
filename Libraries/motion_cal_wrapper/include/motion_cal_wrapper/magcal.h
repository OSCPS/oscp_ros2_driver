#ifndef MAGCAL_H_
#define MAGCAL_H_

#ifdef __cplusplus
extern "C" {
#endif


void motioncal_reset(void);


int motioncal_add_sample(
    int16_t x,
    int16_t y,
    int16_t z
);


void motioncal_refresh_quality(void);

float motioncal_gap_error(void);

float motioncal_fit_error(void);

int motioncal_valid_cal(void);

void motioncal_get_offset(float out[3]);

void motioncal_get_matrix(float out[9]);


int motioncal_sample_count(void);

int MagCal_Run(void);



#ifdef __cplusplus
}
#endif

#endif