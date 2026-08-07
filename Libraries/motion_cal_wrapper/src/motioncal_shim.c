// motioncal_shim.c
//
// Thin shim that exposes the real MotionCal calibration engine
// (magcal.c + matrix.c + quality.c, unmodified) as a clean C API,
// so it can be built as a shared library and called from Python via
// ctypes. This replaces what visualize.c / rawdata.c / gui.cpp used
// to provide (the global `magcal` struct, apply_calibration(), and
// the buffer-management/eviction logic), without pulling in any
// OpenGL, wxWidgets, or serial-port code.

#include "motion_cal_wrapper/imuread.h"

#include <string.h>
#include <stdlib.h>

#include "motion_cal_wrapper/quality.h"
#include "motion_cal_wrapper/magcal.h"

// imuread.h declares `extern MagCalibration_t magcal;` and
// `extern Quaternion_t current_orientation;` -- normally defined in
// visualize.c. We provide them here instead.
MagCalibration_t magcal;
// Quaternion_t current_orientation;

// Same hard/soft-iron transform as visualize.c's apply_calibration(),
// reproduced here so we don't need to link visualize.c (and its GL calls).
void apply_calibration(int16_t rawx, int16_t rawy, int16_t rawz, Point_t *out) {
	float x, y, z;

    x = ((float)rawx) - magcal.V[0];
    y = ((float)rawy) - magcal.V[1];
    z = ((float)rawz) - magcal.V[2];
	out->x = x * magcal.invW[0][0] + y * magcal.invW[0][1] + z * magcal.invW[0][2];
	out->y = x * magcal.invW[1][0] + y * magcal.invW[1][1] + z * magcal.invW[1][2];
	out->z = x * magcal.invW[2][0] + y * magcal.invW[2][1] + z * magcal.invW[2][2];
}

// --- Public API for ctypes ---

void motioncal_reset(void) {
	memset(&magcal, 0, sizeof(magcal));
	magcal.V[2] = 80.0f;  // matches raw_data_reset()'s initial guess
	magcal.invW[0][0] = 1.0f;
	magcal.invW[1][1] = 1.0f;
	magcal.invW[2][2] = 1.0f;
	magcal.FitError = 100.0f;
	magcal.FitErrorAge = 100.0f;
	magcal.B = 50.0f;
	quality_reset();
}

// Add one raw magnetometer sample (int16 counts, same units the
// device sends) into MotionCal's buffer, using its own eviction
// policy (choose_discard-equivalent nearest-neighbor / worst-error
// purge), then run the calibration solver and update quality metrics
// for that point. Returns 1 if a new calibration was accepted this
// call, 0 otherwise.
int motioncal_add_sample(int16_t rawx, int16_t rawy, int16_t rawz) {
	int i, slot = -1;
	Point_t point;

	// find an empty slot first
	for (i = 0; i < MAGBUFFSIZE; i++) {
		if (!magcal.valid[i]) { slot = i; break; }
	}
	if (slot < 0) {
		int64_t minsum = -1;
		int mi = 0, mj = 1;
		for (i = 0; i < MAGBUFFSIZE; i++) {
			for (int j = i + 1; j < MAGBUFFSIZE; j++) {
				int64_t dx = magcal.BpFast[0][i] - magcal.BpFast[0][j];
				int64_t dy = magcal.BpFast[1][i] - magcal.BpFast[1][j];
				int64_t dz = magcal.BpFast[2][i] - magcal.BpFast[2][j];
				int64_t d = dx*dx + dy*dy + dz*dz;
				if (minsum < 0 || d < minsum) { minsum = d; mi = i; mj = j; }
			}
		}
		slot = (random() & 1) ? mi : mj;
	}

	magcal.BpFast[0][slot] = rawx;
	magcal.BpFast[1][slot] = rawy;
	magcal.BpFast[2][slot] = rawz;
	magcal.valid[slot] = 1;

	return MagCal_Run();
}

// quality_update() accumulates state that only makes sense computed
// fresh over the WHOLE current buffer (this is what visualize.c did,
// once per render frame -- quality_reset() then quality_update() for
// every valid point). Calling quality_update() once per incoming
// sample without resetting (as an earlier version of this shim did)
// double-counts evicted points forever and the metrics blow up once
// the buffer fills and eviction starts. Call this before reading any
// of the quality getters below.
void motioncal_refresh_quality(void) {
	int i;
	Point_t point;

	quality_reset();
	for (i = 0; i < MAGBUFFSIZE; i++) {
		if (magcal.valid[i]) {
			apply_calibration(magcal.BpFast[0][i], magcal.BpFast[1][i],
				magcal.BpFast[2][i], &point);
			quality_update(&point);
		}
	}
}

float motioncal_gap_error(void)         { 
	return quality_surface_gap_error(); 
}

float motioncal_variance_error(void)    { 
	return quality_magnitude_variance_error(); 
}

float motioncal_wobble_error(void)      { 
	return quality_wobble_error(); 
}

float motioncal_spherical_fit_error(void) { 
	return quality_spherical_fit_error(); 
}

float motioncal_fit_error(void)         { 
	return magcal.FitError; 
}

int   motioncal_valid_cal(void)         { 
	return magcal.ValidMagCal; 
}

float motioncal_field_strength(void)    { 
	return magcal.B; 
}

void motioncal_get_offset(float out[3]) {
	out[0] = magcal.V[0];
	out[1] = magcal.V[1];
	out[2] = magcal.V[2];
}

void motioncal_get_matrix(float out[9]) {
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			out[i*3+j] = magcal.invW[i][j];
}

int motioncal_sample_count(void) {
	int i, count = 0;
	for (i = 0; i < MAGBUFFSIZE; i++) if (magcal.valid[i]) count++;
	return count;
}