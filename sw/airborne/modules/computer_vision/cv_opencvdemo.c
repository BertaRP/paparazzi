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
 t
*/

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include <string.h>

#define FRAME_RATE 1
#define THRESHOLD_COUNT 10
#define THRESHOLD_TIME 2

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

double *times2contact;
int nFrame = 0;
int start = 1;
int width;
int height;

struct image_t* opencv_func(struct image_t* img);

/* =======================================================================================================================================
 =======================================================================================================================================*/

struct image_t* opencv_func(struct image_t* img)
{

	if (img->type == IMAGE_YUV422)
	{
		// If first time initialize times2contact to all -1
		if (start)
		{
			width = img->w;
			height = img->h;
			int npixels = width*height;
			times2contact = (double *)malloc(npixels*sizeof(double));
			fill_array_with_minus_one(times2contact ,npixels);
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
    return img;
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


/* Looks at times2contact and reaches a decision on where to move :)
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
int corner_avoider_periodic(void)
{
    // CHANGE THIS HEADING DECISION FOR A DECISION MAKER
    // (heading_decision = -1 -> left, heading_decision = 0 -> center, heading_decision = 1 -> right)
    // (heading_decision = -2 -> sharp turn left, heading_decision = 2 -> sharp turn right)
    int heading_decision = 0;
    int nC = 0;
    int nR = 0;
    int nL = 0;
    double timep;
    
    //printf("width %d \n",width);
    //printf("height %d \n",height);

	for (int x = 0; x < width; ++x)
	{
	    for (int y = 0; y < height; ++y)
	    {
	        timep = times2contact[width*y+x];
	        //printf("timep  %1.1f \n", timep);
	        //printf("index %d \n", width*y+x);
	        if (timep == -1)
	        {
	        	continue;
	        }

			if (timep <= THRESHOLD_TIME) 
			{
		    	if (x < width/3.0) {
					nL++; 		// pixel in Left section
		    	} else {
					if (x < 2.0*width/3.0) 
					{
			    		nC++; 	// pixel in Center section
		        	} else {
			    		nR++; 	// pixel in Right section
	    	    	}
		    	}
	  		}
	    }
	}

    if (nC <= THRESHOLD_COUNT) { 
		heading_decision = 0; 		// go straight
    } else {
		if (nR > nL) 
		{
	    	if (nL <= THRESHOLD_COUNT) 
	    	{
				heading_decision = -1; 	// turn left
	    	} else {
				heading_decision = -2; 	// turn sharply left
	    	}
		} else {
	    	if (nR <= THRESHOLD_COUNT) 
	    	{
				heading_decision = 1; 	// turn right
	    	} else {
				heading_decision = 2; 	// turn sharply right
	    	}
		}
    }
    
    printf("Heading_decision: %d \n", heading_decision);

    return heading_decision;
}