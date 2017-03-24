/* 	Copyright: Group 10 AFMAV Course March 2016  */

/* 	This program is the implementation of a monocular vision based object 
	detection approach based on the assumption that objects that are close 
	are more homogeneous than their surroundings.
	
	For a specified window in a grayscale image, the image is partitioned in 
	a number of slices. For each slice the total variance is determined based 
	on the grayscale values of the pixels. Slices with high variance are assumed
	to indicate objects further away from the camera and are therefore safe to 
	fly towards.
	
	The program determines if it is safe to go forward based on the variance of
	the part of the environment in the flight path of the drone.
	This program also determines, based on clusters of high variant slices in the
	image, in which direction the drone should yaw to resume its flight.*/


//----Includes----// //******I don't think we need this*******//
#include "imagevar.h"
#include <stdio.h>
#include <math.h>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;


int imagevar(char* img, int width, int height)
{
	//----Initialize variables----//
	Mat M(width,height, CV_8UC2, img);
	Mat image;

	int crop_h=100;							//Define number of pixels to be cropped on top and bottom
	int crop_w=40;
	int height_c = height - 2*crop_h;
	int width_c = width - 2*crop_w;
	int crop_shift = 30;					//shift the cropped window up with # pixels 
											//(in order to correct for fisheye when looking straight ahead, cam is tilted downwards)
											
	int slice_n = 48;						//number of slices for which variance is calculated
	int dw = width_c / slice_n;				//width of slice in pixels

	double entropy[slice_n];
	

	cvtColor(M, image, CV_YUV2GRAY_Y422); 	//Convert to grayscale


/*!VARIABLES THAT CAN BE EDITED TO TUNE PERFORMANCE!
	slice_n
	crop_h/crop_w
	
	middle_slices[]		//scope for det. homogeneity
	middle_slices_2[]	//scope for det # of high variant slices around image center

	entr_tresh 			//related constant factor               *********
	entropy_treshold		//treshold for image homogeneity    *********
	white_treshold		//number of white slices in middle of screen above which it is safe to go fw (!MAYBE CONVERT TO FRACTION OR %)      ********
*/

    
//----Determine the entropy of each slice----//
    
    //The bigger the entropy the more change there is in the image, meaning a higher variation of the pixel colours (Direct measure of optical flow)
    
    double probability_array[slice_n];
    
    for (int slice=0; slice <slice_n; slice++){
        //Determine entropy of each slice
        for (int i=(crop_w + (slice*dw));i <(crop_w + ((slice+1)*dw)); i++){
            for (int row=(crop_h-crop_shift); row <(height-crop_h-crop_shift); row++){
                //cout<<i<<" "<<j<<" "<<(int)image.at<uchar>(i,j)<<endl;
                int a=(int)image.at<uchar>(i,row);
                int b=(int)image.at<uchar>(i,row-1);
                int x=a-b;
                if(x<0)
                    x=0-x;
                //Count pixels with the same colour
                probability_array[x]++;
                //grey_image.at<uchar>(i,j) = 255;
            }
        }
        
        //calculating probability, divide by the total amount of pixels of the slice (Normalise against total pixel amount)
        int n=dw*height_c;
        for(int i=0;i<256;i++)
        {
            probability_array[i]/=n;
            //cout<<probability_array[i]<<endl;
        }
        // Apply Galileo Imaging Team formula for entropy: Sum(P(i)*log_2(P(i)))
        float entropy[slice];
        entropy[slice]=0;
        for(int i=0;i<256;i++)
        {
            if (probability_array[i]>0)
            {
                float x=probability_array[i]*log(probability_array[i]);
                entropy[slice]+=x;
            }
        }
    }


	
//----Calculate mean and homogeneity in image----//

	double gen_entr_slice[slice_n];
	double gen_entr = 0;
	double means_tot = 0;										//Mean of std deviations of all slices


//----Use when selecting middle part of image for det. gen_stddev----//

	int middle_slices[2];
	int middle_slices_2[2];
	
	//----Define image center for 
	middle_slices[0] = round(slice_n*0.4);						//increase mult factor for broader scope for det. homogeneity
	middle_slices[1] = round(slice_n*0.6);
	int middle_slice_n = middle_slices[1] - middle_slices[0];

	//----
	middle_slices_2[0] = round(slice_n*0.4);					//increase mult. factor for broader scope for det # of high variant slices around image center
	middle_slices_2[1] = round(slice_n*0.6);


	for (int i=middle_slices[0]; i<middle_slices[1]; i++){
	means_tot += entropy[i];							//Sum all standard deviations
	}
	means_tot = means_tot/middle_slice_n;

	for (int i=middle_slices[0]; i<middle_slices[1]; i++){
		gen_entr_slice[i] = (entropy[i] - means_tot)*(entropy[i] - means_tot);
		gen_entr += gen_entr_slice[i];
	}
	gen_entr = sqrt(gen_entr / (middle_slice_n-1));

	
	
//----Find slices with lowest and highest entropy----//

	uint8_t min_entr = entropy[0]; 							//Start search at first entry
	uint8_t max_entr = entropy[0]; 							//Start search at first entry
	int min_slice = 0;
	int max_slice = 0;

	//----Find slices with minimum and maximum variance
	for (int i=1; i<slice_n; i++) {		
		if (entropy[i] < min_entr) {
		min_entr = entropy[i];
				min_slice = i;
		}
	if (entropy[i] > max_entr) {
		max_entr = entropy[i];
				max_slice = i;
		}
	}
	
	int max_x = crop_w + ((max_slice*dw)-(dw/2)); 				//Find x coordinate (center of slice) with largest variance

	

//----Classify brightest x% of slices----//

	uint8_t dif_entr = max_entr - min_entr;
	int tot_entr_cls[slice_n];
	double entr_tresh = 0.6 * dif_entr;							//Treshold for classification of most variant slices, to be tuned !!!!!

	//Classify most variant slices
	for (int i=0; i<slice_n; i++) {	
        	if ((min_entr + entr_tresh) < entropy[i]) {
			tot_entr_cls[i] = 1;
        	}
		else {
			tot_entr_cls[i] = 0;
        	}
	}


	int white_mid_tot=0;

	for (int i=middle_slices_2[0]; i<middle_slices_2[1]; i++){
		white_mid_tot += tot_entr_cls[i];					//Sum all standard deviations
	}

	
	
//----Find largest bright cluster in image and determine its center location----//

	int cls[slice_n];
	int max_cls = 0;
	int max_cls_i = 0;
	
	cls[0] = 0;

	for(int i=1; i<slice_n; i++){						//Find clusters of most variant image slices and respective sizes of these clusters
		if(tot_entr_cls[i] == 1){
			cls[i] = (cls[(i-1)] + 1);
			if(cls[i] > max_cls){
				max_cls = cls[i];						//Determine size of largest cluster
				max_cls_i = i;							//Determine slice number (i) of most right slice in largest cluster 
			}
		}
		else{
			cls[i] = 0;
		}
	}

	double cg_cls_x = 0;
	cg_cls_x = crop_w + round(dw*((max_cls_i) - (max_cls/2)));	//Determine center of largest cluster of most variant image slices

	int dir_turn=0;
	if (cg_cls_x > (crop_w+(width_c/2))){
		dir_turn = 1;									//Make right turn
	}
	else{
		dir_turn = 0;									//Make left turn
	}

	

//----CHECK IF SAFE TO GO FORWARD----//

	int sfw=0;											//If sfw=1 then it is safe to go forward
	double entropy_treshold = 1.2;						//Treshold for overall entropy
	int white_treshold = 4;								//Treshold for the number of slices with high entropy in defined image center
	//int circle_color;									//Use if circle is displayed on image

	//----Check if number of high variant slices in image center are higher than treshold
	if (white_mid_tot > white_treshold){
		//----Check if homogeneity in image center is higher than treshold
		if(gen_entr > entropy_treshold){
			sfw=1;
			//circle_color=255;		//Circle if white if sfw=1 (Use if circle is displayed on image)
		}
		else{
			sfw=0;
			//circle_color=0;		//Circle if black if sfw=1 (Use if circle is displayed on image)
		}
		
	}
	else{
		sfw = 0;
		//circle_color=0;
	}


//----PRINT VARS. IN TERMINAL----//

	//printf("gen_stddev: %f \n", gen_stddev);
	//printf("gen_stddev: %f , means_tot %f , min_std: %d , max_std: %d \n", gen_stddev, means_tot, min_std, max_std);
	//printf("dif_std: %d \n", dif_std);
	//printf("Bright_tresh: %f \n tot_std_dev[5]: %d",bright_tresh,tot_std_dev[5]);
	//printf("Max_x: %d\n",max_x);
	//printf("White_mid_tot %d\n gen_stddev: %f\n cg_cls_x: %f\n",white_mid_tot,gen_stddev,cg_cls_x);
	//printf("white_mid_tot %d\n gen_stddev: %f\n",white_mid_tot,gen_stddev);
	//printf("max_cls_i: %d\n cg_cls_x: %d\n",max_cls_i,cg_cls_x);

	

//----TO DISPLAY IMAGE WITH CIRCLE OVER RTP STREAM----//

//----Define circle location on image
	//Point2f cg(max_x,((height/2) - crop_shift)); 				//Make point object with floats at slice location with highest variance
	/*Point2f cg(cg_cls_x,((height/2) - crop_shift));
	Point2f cg_c((crop_w + width_c/2),((height/2) - crop_shift));  */
				

//----Overwrite image on drone with edited image
	//RNG rng(12345);
	//circle(image, cg, 5, Scalar(circle_color, circle_color, circle_color), -1, 8, 0 );
	//circle(image, cg, 5, Scalar(0,0,0), 1, 8, 0 );
	//circle(image, cg_c, 5, Scalar(circle_color, circle_color, circle_color), -1, 8, 0 );
	//cvPutText(image, const char* text, cg, const CvFont* font, Scalar(0, 0, 0))

	/*for (int n=0; n < slice_n; n++){
			for (int j=(crop_w+(n*dw)); j <(crop_w+((n+1)*dw)); j++){
				for (int row=(crop_h-crop_shift); row <(height-crop_h-crop_shift); row++){
					//img[(row*width+j)*2+1] = tot_std_dev[n];
					img[(row*width+j)*2+1] = (tot_std_dev_cls[n] * 255);
					//img[(row*width+j)*2+1] = image.at<uint8_t>(row,j);
					img[(row*width+j)*2] = 127;
				}
			}
		}*/


//----RETURN VARIABLES----------------------------------//

	int ret=2;	//Set standard to not safe to go forward + right turn
	
	//----Ascribe value of ret variable to corresponding case----//
	
	if ((sfw==1 && dir_turn==1)) {		//Safe to go forward + make right turn
		ret = 1;
	}
	else if((sfw==0 && dir_turn==1)) {  //Not safe to go forward + make right turn
		ret = 2;
	}
	else if((sfw==1 && dir_turn==0)) {	//Safe to go forward + make left turn
		ret = 3;
	}
	else if((sfw==0 && dir_turn==0)){	//not safe to go forward + make left turn
		ret = 4;
	}


return ret;
}

