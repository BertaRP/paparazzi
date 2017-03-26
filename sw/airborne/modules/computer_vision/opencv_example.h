/*
 blablabla finish header introduction
 */

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif
    
extern void image_pipeline(char* img, int width, int height, double* times2contact);
extern void image_pipeline_init(char* img, int width, int height);
extern void fill_array_with_minus_one(double *array, int npixels);

extern double min_time2contact;

#ifdef __cplusplus
}
#endif

#endif

