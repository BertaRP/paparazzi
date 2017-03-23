/*
 blablabla finish header introduction
 */

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif
    
    
// Functions to be used by opticflow_calculator
extern void opticFlow_init(struct image_t* img);
extern void opticFlow_periodic(struct image_t* img);

#ifdef __cplusplus
}
#endif

#endif

