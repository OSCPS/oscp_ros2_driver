#ifndef QUALITY_H_
#define QUALITY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "motion_cal_wrapper/imuread.h"


void quality_reset(void);

void quality_update(const Point_t *point);

float quality_surface_gap_error(void);

float quality_magnitude_variance_error(void);

float quality_wobble_error(void);

float quality_spherical_fit_error(void);


#ifdef __cplusplus
}
#endif

#endif