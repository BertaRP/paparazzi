/* CV_OPENCVDEMO.c
* 
* Includes the definition of the image pipeline
* and its initialization for the first frame. 
* Also, the heading decision is defined in corner_avoider_periodic, 
* which will be called in orange_avoider.c
*/

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include <string.h>

#define FRAME_RATE 1
#define THRESHOLD_COUNT 10
#define THRESHOLD_TIME 0.01

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


/*
Calls the image_pipeline function from opencv_example.cpp, which will perform all the 
image processing operations. 
	img: image from the front camera
return img: image after the image processing process 
*/
struct image_t* opencv_func(struct image_t* img)
{
	if (img->type == IMAGE_YUV422)
	{
		// If first time initialize times2contact to all -1
		if (start)
		{
			height = img->w;
			width = img->h;
			int npixels = width*height;
			times2contact = (double *)malloc(npixels*sizeof(double));
			fill_array_with_minus_one(times2contact ,npixels);
			image_pipeline_init((char *) img->buf, width, height);
			start = 0;
		} 

		if (nFrame % FRAME_RATE == 0)
		{
    		image_pipeline((char *) img->buf, width, height, times2contact);
		}

    	// Add one to the frame count
    	nFrame++;
    }
    return img;
}



/*
Connects the front camera with the function opencv_func
*/
void opencvdemo_init(void)
{

 	// Init times2contact
 	times2contact = NULL;

 	// Link image pipeline to "camera pipe" (Outputs a frame and executes the pipeline every frame)
    cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func);
}


/*
Returns a heading decision taking into account the times to contact
in the three parts in which the image from camera is divided
*/
int corner_avoider_periodic(void)
{
    int heading_decision = 0;
    int nC = 0;
    int nR = 0;
    int nL = 0;
    double timep;
    
	for (int x = 0; x < width; ++x)
	{
	    for (int y = 0; y < height; ++y)
	    {
	        timep = times2contact[width*y+x];

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
    
    return heading_decision;
}