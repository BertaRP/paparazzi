/*
 * OBSTACLE AVOIDER
 * 
 *
 * This file is based on the colorfilter.c provided in Paparazzi. 
 * 
 * This program crops the image from camera in three, and counts the 
 * orange and black pixels in each part of the image (left, center and right)
 * This updates different counters that will be used to determine the heading
 * decision (the one based on the colors)
 *
 */
 
#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void colorfilter_init(void);

extern uint8_t color_lum_minO;
extern uint8_t color_lum_maxO;

extern uint8_t color_cb_minO;
extern uint8_t color_cb_maxO;

extern uint8_t color_cr_minO;
extern uint8_t color_cr_maxO;

extern uint8_t color_lum_minB;
extern uint8_t color_lum_maxB;

extern uint8_t color_cb_minB;
extern uint8_t color_cb_maxB;

extern uint8_t color_cr_minB;
extern uint8_t color_cr_maxB;

extern uint16_t color_countOl, color_countOc, color_countOr;
extern uint16_t color_countBl, color_countBc, color_countBr;

extern struct video_listener *listener;

#endif /* COLORFILTER_CV_PLUGIN_H */
