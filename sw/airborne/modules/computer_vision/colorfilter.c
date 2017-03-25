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
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_minO = 105;
uint8_t color_lum_maxO = 205;
uint8_t color_cb_minO  = 52;
uint8_t color_cb_maxO  = 140;
uint8_t color_cr_minO  = 180;
uint8_t color_cr_maxO  = 255;
uint8_t color_lum_minB = 105;
uint8_t color_lum_maxB = 205;
uint8_t color_cb_minB  = 52;
uint8_t color_cb_maxB  = 140;
uint8_t color_cr_minB  = 180;
uint8_t color_cr_maxB  = 255;

// Result
int color_countO = 0, color_countB = 0;

// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  struct image_t* orange;
  struct image_t* black;

  orange = (struct image_t *)malloc(sizeof(struct image_t));
  black = (struct image_t *)malloc(sizeof(struct image_t));

  image_create(orange, img->w, img->h, img->type);
  image_create(black, img->w, img->h, img->type);

  image_copy(img, orange);
  image_copy(img, black);


  // Filter
  color_countB = image_yuv422_colorfilt(img, black,
                                       color_lum_minB, color_lum_maxB,
                                       color_cb_minB, color_cb_maxB,
                                       color_cr_minB, color_cr_maxB
                                      );
  color_countO = image_yuv422_colorfilt(img, orange,
                                       color_lum_minO, color_lum_maxO,
                                       color_cb_minO, color_cb_maxO,
                                       color_cr_minO, color_cr_maxO
                                      );
  image_free(orange);
  image_free(black);

  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}
