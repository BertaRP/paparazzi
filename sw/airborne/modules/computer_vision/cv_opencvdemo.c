/* OPTICFLOW_FGBG
 
 This module blabalbla finish this introduction
 
 (INIT)
 .- orange_avoider_init
 .- blur (img) --> median_0
 .- gray (median_0) --> gray_blurred_0
 .- fgbgMOG2 (median_0) --> fgmask_0
 .- cornerDetection (gray_blurred_0, fgmask_0) --> tracking_pts_0
 
 (PERIODIC)
 .- blur (img) --> median
 .- gray (median) --> gray_blurred
 .- opticFlow (gray_blurred, gray_blurred_0, tracking_pts_0) --> tracking_pts
 .- time2contact (tracking_pts, tracking_pts_0) --> optical_flow_count
 
 .- fgbgMOG2 (median) --> fgmask
 .- cornerDetection (gray_blurred, fgmask) --> tracking_pts_0
 
*/

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include <string.h>

#define FRAME_RATE 1

double *times2contact;
int nFrame = 0;
int start = 1;


struct image_t* opencv_func(struct image_t* img);
int take_decision_periodic(void);

/* =======================================================================================================================================
 =======================================================================================================================================*/

struct image_t* opencv_func(struct image_t* img)
{

	if (img->type == IMAGE_YUV422)
	{
		// If first time initialize times2contact to all -1
		if (start)
		{
			int npixels = img->w*img->h;
			times2contact = (double *)malloc(npixels*sizeof(double));
			memset(times2contact, -1, npixels*sizeof(double));
			image_pipeline_init((char *) img->buf, img->w, img->h);
			start = 0;
		}

		if (nFrame % FRAME_RATE == 0)
		{
			// TODO: Check if necesary to give width and height
    		image_pipeline((char *) img->buf, img->w, img->h, times2contact);
		}

    	// Add one to the frame count
    	nFrame++;
    }
    return NULL;
}



/* =======================================================================================================================================
 =======================================================================================================================================*/

void opencvdemo_init(void)
{

 	// Init times2contact
 	times2contact = NULL;

 	// Link image pipeline to "camera pipe" (Outputs a frame and executes the pipeline every frame)
    cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func);
}

int take_decision_periodic(void)
{
	/* BLANCA:
	 Looks at times2contact and reaches a decision on where to move :)
	 times2contact shape: 
 	|time1  time2  time3  time 4 |
 	|  .      .      .      .    |
 	|  .      .      .      .    |
 	|  .      .      .      .    |
 	|  .      .      .    time(n)|
	

	The times are placed in the exact pixels of the image --> make submatrices (I would start with 3) -->
	find the minimum time of those submatrices --> DIRECTION TO AVOID! 
	Points where the time to contact is calculated will be labeled as -1.
	

*/

	return 0;
}