/* 	Copyright: Group 10 AFMAV Course
	March 2016
*/


#include "modules/computer_vision/cv.h"
#include "modules/group10/variance_calculator.h"
#include "modules/group10/imagevar.h"

//----Initialize variables----//
int safe_fw = 0;
int ret_x = 2;
int ret_r = 0;

// Function
struct bool_t* opencv_func(struct image_t* img); //Declare
struct bool_t* opencv_func(struct image_t* img)  //Initialise
{

  if (img->type == IMAGE_YUV422)
  {
	//----Returned variable from imagevar function---//
	ret_x = imagevar((char*) img->buf, img->w, img->h);

	//----Relate value of ret_x to corresponding case----//
	if (ret_x==1) {
		safe_fw=1; 
		ret_r=1;
	}
	else if(ret_x==2) {
		safe_fw=0;
		ret_r=1;
	}
	else if(ret_x==3) {
		safe_fw=1; 
		ret_r=0;
	}
	else if(ret_x==4){
		safe_fw=0;
		ret_r=0;
	}
  }

  return FALSE;
}

void variance_calculator_init(void)
{
  cv_add(opencv_func);
}


