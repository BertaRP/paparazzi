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
// Orange filter parameters
uint8_t color_lum_minO = 105;
uint8_t color_lum_maxO = 205;
uint8_t color_cb_minO  = 52;
uint8_t color_cb_maxO  = 140;
uint8_t color_cr_minO  = 180;
uint8_t color_cr_maxO  = 255;
// Black filter parameters
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


/*
Calculates the number of black and orange pixels in the 
image, divided in left, center and right regions
  img: pointer to the image from camera 
*/
struct image_t *colorfilter_func(struct image_t *img)
{
  // Declare images
  struct image_t* orangeL;
  struct image_t* orangeC;
  struct image_t* orangeR;
  struct image_t* blackL;
  struct image_t* blackC;
  struct image_t* blackR;

  // Allocate images
  orangeL = (struct image_t *)malloc(sizeof(struct image_t));
  orangeC = (struct image_t *)malloc(sizeof(struct image_t));
  orangeR = (struct image_t *)malloc(sizeof(struct image_t));
  blackL = (struct image_t *)malloc(sizeof(struct image_t));
  blackC = (struct image_t *)malloc(sizeof(struct image_t));
  blackR = (struct image_t *)malloc(sizeof(struct image_t));

  // Create images
  image_create(orangeL, img->w, img->h/3, img->type);
  image_create(orangeC, img->w, img->h/3, img->type);
  image_create(orangeR, img->w, img->h/3, img->type);
  image_create(blackL, img->w, img->h/3, img->type);
  image_create(blackC, img->w, img->h/3, img->type);
  image_create(blackR, img->w, img->h/3, img->type);

  //VERBOSE_PRINT("Height image: %d  and height image_crop: %d \n", img->h, img->h/3);

  // Division of the images for the orange filter
  orangeL = image_crop_func(img, 0);
  orangeC = image_crop_func(img, 1);
  orangeR = image_crop_func(img, 2);
  //VERBOSE_PRINT("Image cropped: width =  %d heigth = %d \n", orangeL->w, orangeL->h);

  // Copy the just created images for the black filter
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

/*
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

  // Free the images
  image_free(orangeL);
  image_free(orangeC);
  image_free(orangeR);
  image_free(blackL);
  image_free(blackC);
  image_free(blackR);

  return img; // Colorfilter did not make a new image
}


/*
Crops the input image in three images of width = width/3
  *input: pointer to the image
  n: identifier  (n = 0 (left), n = 1 (center) and n = 2 (right))
*/
struct image_t *image_crop_func(struct image_t *input, int n)
{
    // Declare image
   	struct image_t* img2;
    // Allocate image
    img2 = (struct image_t *)malloc(sizeof(struct image_t));
    // Properties of the cropped image
    int width = input->w; // Width of the cropped image
	  int height = input->h/3;

    // Create image
    image_create(img2, width, height, input->type);

    uint8_t *input_buf = (uint8_t *)input->buf;
    uint8_t *output_buf = (uint8_t *)img2->buf;
	

    if (n==2) {
    	for (int i = n*height; i < (n+1)*height; i++) {
          for (int j = 0; j < width; j++) {   

    		    output_buf[i*(width) + j - n*height*width] = input_buf[i*width + j];
        }
      }	
    } else {    

    	   for (int i = n*height; i < (n+1)*height; i++) {
            for (int j = 0; j < width; j++) {   

    	         output_buf[i*width + j - n*height*width] = input_buf[i*width + j];
        }
      }
    }

  img2->buf = output_buf;

  return img2; // Cropped image in side (n = 0 (left), n = 1 (center) and n = 2 (right))
}

/*
Initializes the color filter ot connect the front camera with colorfilter_func
*/
void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}
