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

#define COLOR_FILTER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if COLOR_FILTER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

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
uint16_t color_countOl = 0, color_countOc = 0, color_countOr = 0;
uint16_t color_countBl = 0, color_countBc = 0, color_countBr = 0;

// Function Crop
struct image_t *image_crop_func(struct image_t *img_crop, int n);

// Function ColorFilter
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
/*  struct image_t* orange;
  struct image_t* black;

  orange = (struct image_t *)malloc(sizeof(struct image_t));
  black = (struct image_t *)malloc(sizeof(struct image_t));
  image_create(orange, img->w, img->h, img->type);
  image_create(black, img->w, img->h, img->type); 

  image_copy(img, orange);
  image_copy(img, black); */

  //VERBOSE_PRINT("In the beginning of times \n");

  struct image_t* orangeL;
  struct image_t* orangeC;
  struct image_t* orangeR;
  struct image_t* blackL;
  struct image_t* blackC;
  struct image_t* blackR;

  orangeL = (struct image_t *)malloc(sizeof(struct image_t));
  orangeC = (struct image_t *)malloc(sizeof(struct image_t));
  orangeR = (struct image_t *)malloc(sizeof(struct image_t));
  blackL = (struct image_t *)malloc(sizeof(struct image_t));
  blackC = (struct image_t *)malloc(sizeof(struct image_t));
  blackR = (struct image_t *)malloc(sizeof(struct image_t));

  image_create(orangeL, img->w, img->h/3, img->type);
  image_create(orangeC, img->w, img->h/3, img->type);
  image_create(orangeR, img->w, img->h/3, img->type);
  image_create(blackL, img->w, img->h/3, img->type);
  image_create(blackC, img->w, img->h/3, img->type);
  image_create(blackR, img->w, img->h/3, img->type);

  //VERBOSE_PRINT("Height image: %d  and height image_crop: %d \n", img->h, img->h/3);

  // Division of Orange and Black images in three
  orangeL = image_crop_func(img, 0);
  orangeC = image_crop_func(img, 1);
  orangeR = image_crop_func(img, 2);
  //VERBOSE_PRINT("Image cropped: width =  %d heigth = %d \n", orangeL->w, orangeL->h);

  image_copy(orangeL, blackL);
  image_copy(orangeC, blackC);
  image_copy(orangeR, blackR);

  // Color count for each small image
  color_countBl = image_yuv422_colorfilt(blackL, blackL,	// Black left
                                       color_lum_minB, color_lum_maxB,
                                       color_cb_minB, color_cb_maxB,
                                       color_cr_minB, color_cr_maxB
                                      );
  //VERBOSE_PRINT("Black count left: %d", color_countBl);
  color_countBc = image_yuv422_colorfilt(blackC, blackC,	// Black center
                                       color_lum_minB, color_lum_maxB,
                                       color_cb_minB, color_cb_maxB,
                                       color_cr_minB, color_cr_maxB
                                      );
  //VERBOSE_PRINT("Black count center: %d", color_countBc);
  color_countBr = image_yuv422_colorfilt(blackR, blackR,	// Black right
                                       color_lum_minB, color_lum_maxB,
                                       color_cb_minB, color_cb_maxB,
                                       color_cr_minB, color_cr_maxB
                                      );
  //VERBOSE_PRINT("Black  count: left = %d, center = %d, right = %d \n", color_countBl, color_countBc, color_countBr);

  color_countOl = image_yuv422_colorfilt(orangeL, orangeL,	// Orange left
                                       color_lum_minO, color_lum_maxO,
                                       color_cb_minO, color_cb_maxO,
                                       color_cr_minO, color_cr_maxO
                                      );
  color_countOc = image_yuv422_colorfilt(orangeC, orangeC,	// Orange left
                                       color_lum_minO, color_lum_maxO,
                                       color_cb_minO, color_cb_maxO,
                                       color_cr_minO, color_cr_maxO
                                      );
  color_countOr = image_yuv422_colorfilt(orangeR, orangeR,	// Orange left
                                       color_lum_minO, color_lum_maxO,
                                       color_cb_minO, color_cb_maxO,
                                       color_cr_minO, color_cr_maxO
                                      );
  //VERBOSE_PRINT("Orange count: left = %d, center = %d, right = %d \n", color_countOl, color_countOc, color_countOr);

/*  // Filter
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
  image_free(black);*/

  /*uint16_t count_nothing = 0;

  count_nothing = image_yuv422_colorfilt_OandB(img, img,	// To modify th image (orange and blue points)
                                       color_lum_minO, color_lum_maxO,
                                       color_cb_minO, color_cb_maxO,
                                       color_cr_minO, color_cr_maxO,
                                       color_lum_minB, color_lum_maxB,
                                       color_cb_minB, color_cb_maxB,
                                       color_cr_minB, color_cr_maxB
                                      );*/

  image_free(orangeL);
  image_free(orangeC);
  image_free(orangeR);
  image_free(blackL);
  image_free(blackC);
  image_free(blackR);

  return img; // Colorfilter did not make a new image
}



struct image_t *image_crop_func(struct image_t *input, int n)
{
   	struct image_t* img2;
    img2 = (struct image_t *)malloc(sizeof(struct image_t));
    int width = input->w; // Width of the cropped image
	int height = input->h/3;


    image_create(img2, width, height, input->type);

  uint8_t *input_buf = (uint8_t *)input->buf;
  uint8_t *output_buf = (uint8_t *)img2->buf;
	
//VERBOSE_PRINT("Before bucle. Height image cropped: %d \n" , height);
if (n==2) {
/*VERBOSE_PRINT("We are in n = %d \n", n);
VERBOSE_PRINT("We have width = %d and height = %d \n", width, height);
VERBOSE_PRINT("We have j0 = %d and jend = %d \n", 0, width);
VERBOSE_PRINT("We have i0 = %d and iend = %d \n", n*height, (n+1)*height);*/
	for (int i = n*height; i < (n+1)*height; i++) {
for (int j = 0; j < width; j++) {
//		VERBOSE_PRINT("Index: %d \n", i*(width) + j - n*height*width);
//VERBOSE_PRINT("We are in n = %d \n", n);
		output_buf[i*(width) + j - n*height*width] = input_buf[i*width + j];
    }
  }	
} else {
/*VERBOSE_PRINT("We are in n = %d \n", n);
VERBOSE_PRINT("We have width = %d and height = %d \n", width, height);
VERBOSE_PRINT("We have j0 = %d and jend = %d \n", 0, width);
VERBOSE_PRINT("We have i0 = %d and iend = %d \n", n*height, (n+1)*height);*/
	for (int i = n*height; i < (n+1)*height; i++) {
for (int j = 0; j < width; j++) {
//VERBOSE_PRINT("We are in index = %d \n", i*(width) + j - n*width);
	output_buf[i*width + j - n*height*width] = input_buf[i*width + j];
    }
  }
}

img2->buf = output_buf;
//VERBOSE_PRINT("After bucle\n");

  return img2; // Cropped image in side (n = 0 (left), n = 1 (center) and n = 2 (right))
}


void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}
