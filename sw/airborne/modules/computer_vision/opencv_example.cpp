/* IMG_PROCESSING
 
 This module blablabla finish this introduction
 
 */

#include "opencv_example.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"
#include <cmath.h>



// Global variables
Ptr<BackgroundSubtractor> pMOG2;

Mat tracking_pts_0;
Mat gray_blurred_0;

/* =======================================================================================================================================
 =======================================================================================================================================*/


void image_pipeline(char* img, double* times2contact)
{

    /* Periodic function to compute the optical flow in an outer loop:
     .- Gets the old image and tracking points (from last frame)
     .- Blur the image
     .- Converts it to grayscale
     .- Create foreground mask (foreground-background segmentation)
     .- Compute optical flow (returns the moving points in the image)
     .- Time2contact
     .- Determine heading direction depending on the time to contact
     .- Recompute the corners--> save corners and current frame to the output to be used in the next iteration
     .- Returns the heading decision (e.g. 0: far left, 1: left, 2: center, 3: right, 4: far right)
     */
    
    Mat median;
    Mat gray_blurred;
    Mat fgmask;
    Mat new_corners;
    Mat tracking_pts;

    Mat image(CV_8UC2, img);    // Implement image pipeline
        
    // Blurs the image with a median filter
    median = medianBlurring(Mat image);
    
    // Convert the blurred image to grayscale
    gray_blurred = grayScl(Mat median);

    // Compute the foreground mask
    fgmask = fgbgMOG2(Mat median);
    
    // Compute the optical flow in the blurred, grayscale image
    tracking_pts = fgbgOpticFlow(Mat gray_blurred, Mat gray_blurred_0, Mat tracking_pts_0);
    
    // Time to contact (or any other decision maker)
    time_vector = time2contact(Mat tracking_pts_0, Mat tracking_pts, Mat gray_blurred);

    // Convert the time_vector to a matrix
    // blablabla
    
    // Detects the new corners on the image (Shi-Tomasi)
    new_corners = cornerDetection(Mat gray_blurred, Mat fgmask);

    // Swap tracking_pts_0 and gray_blurred_0 with current ones
    tracking_pts_0.reshape(new_corners.channels,new_corners.rows);
    gray_blurred_0.reshape(gray_blurred.channels,gray_blurred.rows);

    tracking_pts_0 = new_corners;
    gray_blurred_0 = gray_blurred;

}


void image_pipeline_init(char *img)
{
    Mat image(CV_8UC2, img);

    gray_blurred_0.reshape(image.channels,image.rows);
    tracking_pts_0.reshape(image.channels,image.rows);

}

/* =======================================================================================================================================
 =======================================================================================================================================*/


Mat medianBlurring(Mat image)
{
    // Variables
    int kernel_size = 29;
    
    // Output
    Mat median;

    // Blur it with a median filter (image == input, median == output, 3rd arg == size of kernel (ODD!!!!))
    medianBlur(img, median, kernel_size);
    
    // Output
    return median;
}

/* =======================================================================================================================================
 =======================================================================================================================================*/

Mat grayScl(Mat median)
{
    // Output
    Mat gray_blurred;
    
    // Convert the image (the median) to grayscale
    cvtColor(median, gray_blurred, CV_YUV2GRAY_Y422);
    
    return gray_blurred;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


Mat fgbgMOG2(Mat median)
{
    // Variables
    int     history       = 1;
    double  varThreshold  = 60;
    bool    detectShadows = false;
    
    //Output
    Mat fgmask;
    
    
    // Create the background subtractor
    pMOG2 = createBackgroundSubtractorMOG2(history, varThreshold, detectShadows); //MOG2 approach
    
    // Create the foreground mask (median == input, fgmask == output)
    pMOG2->apply(median,fgmask);
    
    // Output
    return fgmask;
    
}


/* =======================================================================================================================================
=======================================================================================================================================*/


Mat cornerDetection(Mat gray_blurred, Mat fgmask)
{
    // Variables
    int     maxCorners        = 1000;
    double  qualityLevel      = 0.01;
    double  minDistance       = 7;
    int     blockSize         = 7;
    bool    useHarrisDetector = false;
    double  k_harris          = 0.04;
    
    // Output
    Mat tracking_pts_0;

    // Find corners
    goodFeaturesToTrack(gray_blurred, tracking_pts_0, maxCorners, qualityLevel, minDistance, mask = fgmask, blockSize, useHarrisDetector, k_harris);
    
    // Output
    return tracking_pts_0;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


Mat fgbgOpticFlow(Mat gray_blurred, Mat gray_blurred_0, Mat tracking_pts_0)
{
    // Variables
    Mat          status;
    Mat          err;
    Size         winSize         = (13,13);
    int          maxLevel        =  3;
    TermCriteria criteria        = Termcriteria(Termcriteria::COUNT + TermCriteria::EPS, 30, 0.01); // Termination criteria (iterations, delta)
    int          flags           = 0;
    double       minEigThreshold = 0.0001;
    
    // Output
    Mat tracking_pts;
    
    // Optical flow (Lucas-Kanade)
    calcOpticalFlowPyrLK(gray_blurred_0, gray_blurred, tracking_pts_0, tracking_pts, status, err, winSize, maxLevel, criteria, flags, minEigThreshold);
    
    // Output
    return tracking_pts;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/

double time2contact(Mat tracking_pts_0, Mat tracking_pts, Mat gray_blurred)
{
/* This function computes the time to contact given the tracking points (corners) of the previous frame and 
 the points returned by the optical flow calculation for the current frame. 
 
 This is the actual time to contact since it is multiplied by the refresh rate of the function (15 Hz). If this 
 changes, be aware that in "time[idx2] = distance/velocity * 15", 15 should be changed to the new refresh rate. 
 */
    
    int width  = gray_blurred.cols;             // width of the image
    int height = gray_blurred.rows;             // height of the image
    double center[2];                           // position of the center of the image
    double position_x, position_y, distance;    // posision components and distance (module of the position)
    double velocity_x, velocity_y, velocity;    // velocity components and module of the velocity
    double time_vector[height];                        // Time to contact [s]
       
    // Calculate the position of the center of the image
    center   = {0.5 * width, 0.5 * height};
    
    // Time to contact calculation (distance/velocity) * frequency
    for (int idx = 0; idx = tracking_pts_0.rows; idx++)
    {
        for (int idx2 = 0; idx2 = tracking_pts.cols; idx2++)
            {
                position_x = center[0] - tracking_pts_0[0][idx2];
                position_y = center[1] - tracking_pts_0[idx][1];
                
                distance = hypot(position_x, position_y);
                
                velocity_x = tracking_pts[0][idx2] - tracking_pts_0[0][idx2];
                velocity_y = tracking_pts[idx][1] - tracking_pts_0[idx][1];
                
                velocity = hypot(velocity_x,velocity_y);
                
                time_vector[idx2] = distance/velocity * 15;
            }
    }
        
    return time_vector; 
    
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


void opticFlow_init(struct image_t* img)
{
    /* Function to initialize the optical flow process:
     .- Blur the image
     .- Converts it to grayscale
     .- Create foreground mask (foreground-background segmentation)
     .- Corner detection
     .- Saves all to a pointer to be used for the opticFlow_periodic function
     */
    
    Mat median_0;
    Mat gray_blurred_0;
    Mat fgmask;
    Mat tracking_pts_0;


    // Blurs the image with a median filter --> Output = median
    median_0 = medianBlurring(char *img);
    
    // Convert the blurred image to grayscale --> Output = gray_blurred
    gray_blurred_0 = grayScl(Mat median_0);
    
    // Creates a foreground mask via background segmentation --> Output = fgmask
    fgmask = fgbgMOG2(Mat median_0);
    
    // Detects the corners on the image (Shi-Tomasi) --> Ouput = tracking_pts_0
    tracking_pts_0 = cornerDetection(Mat gray_blurred, Mat fgmask);
    
    // Output structure
    opticFlow_output.gray_blurred_0 = gray_blurred_0; // Current image
    opticFlow_output.tracking_pts_0 = tracking_pts_0; // Current corners
    
    
}

/* =======================================================================================================================================
 =======================================================================================================================================*/



int opticFlow_periodic(struct image_t* img);
{
    /* Periodic function to compute the optical flow in an outer loop:
     .- Gets the old image and tracking points (from last frame)
     .- Blur the image
     .- Converts it to grayscale
     .- Create foreground mask (foreground-background segmentation)
     .- Compute optical flow (returns the moving points in the image)
     .- Time2contact
     .- Determine heading direction depending on the time to contact
     .- Recompute the corners--> save corners and current frame to the output to be used in the next iteration
     .- Returns the heading decision (e.g. 0: far left, 1: left, 2: center, 3: right, 4: far right)
     */
    
    Mat median;
    Mat gray_blurred;
    Mat fgmask;
    Mat tracking_pts_0;
    Mat gray_blurred_0;
    Mat new_corners;
    
    
    // Output
    int heading_decision;
    
    tracking_pts_0 = opticFlow_out.tracking_pts_0;
    gray_blurred_0 = opticFlow_out.gray_blurred_0;
    
    
    // Blurs the image with a median filter --> Output = median
    median = medianBlurring(char *img);
    
    // Convert the blurred image to grayscale --> Output = gray_blurred
    gray_blurred = grayScl(Mat median);
    
    // Compute the optical flow in the blurred, grayscale image
    tracking_pts = fgbgOpticFlow(Mat gray_blurred, Mat gray_blurred_0, Mat tracking_pts_0);
    
    // Time to contact (or any other decision maker)
    heading_decision = time2contact(Mat tracking_pts_0, Mat tracking_pts, Mat gray_blurred);
    
    // Recompute the foreground mask --> Output = fgmask
    fgmask = fgbgMOG2(Mat median);
    
    // Detects the new corners on the image (Shi-Tomasi) --> Ouput = tracking_pts_0
    new_corners = cornerDetection(Mat gray_blurred, Mat fgmask);
    
    // Output structure
    opticFlow_out.gray_blurred_0 = gray_blurred_0; // Current image
    opticFlow_out.tracking_pts_0 = new_corners;    // Current corners

    
    return heading_decision;
}
    
