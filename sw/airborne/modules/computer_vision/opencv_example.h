/*
 * IMAGE PROCESSING
 * 
 * This module implements the required functions to detect obstacles and 
 * calculate its time to contact. 
 * If SPARSE = 1, Lucas-Kanade sparse optical flow is used, and the process is: 
 *   .- Gets the old image and tracking points (from last frame)
 *   .- Blurs the image
 *   .- Converts it to grayscale
 *   .- Corner detection
 *   .- Compute optical flow (Lucas-Kanade)
 *   .- Time to contact
 *   
 * If SPARSE = 0, Farneback dense optical flow is used
 *   .- Gets the old image and tracking points (from last frame)
 *   .- Canny edge detection
 *   .- Compute optical flow (Farneback)
 *   .- Time to contact
 *
 */     

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif
    
extern void image_pipeline(char* img, int width, int height, double* times2contact);
extern void image_pipeline_init(char* img, int width, int height);
extern void fill_array_with_minus_one(double *array, int npixels);

float mintime2contact_l;
float mintime2contact_c;
float mintime2contact_r; 

#ifdef __cplusplus
}
#endif

#endif

