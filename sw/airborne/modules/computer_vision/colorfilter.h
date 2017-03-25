/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.h
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

extern int color_countO, color_countB;

extern struct video_listener *listener;

#endif /* COLORFILTER_CV_PLUGIN_H */
