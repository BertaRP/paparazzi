/* IMG_PROCESSING
 
 This module blablabla finish this introduction
 
 */

#include "opencv_example.h"
#include <opencv2/video.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"
#include <cmath>


using namespace std;
using namespace cv;

// Refresh rate of the pipeline
#define REFRESH_RATE 30 // fps

// Blur parameter
#define KERNEL_SIZE 29

// Background substraction parameters
#define HISTORY 1
#define VAR_THRESHOLD 60
static const bool detect_shadows = false;

// Corner detection parameters
#define MAX_CORNERS 1000
#define QUALITY_LEVEL 0.01
#define MIN_DISTANCE 7
#define BLOCK_SIZE 7
static const bool harris_detector = false;
#define K_HARRIS 0.04

// Optic Flow parameters
#define MAX_LEVEL 3
#define FLAGS 0
#define MIN_EIG_THRESHOLD 0.0001
static const TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01); // Termination criteria (iterations, delta)
static const Size winSize = Size(13,13);

vector<Point2f> tracking_pts_0;
Mat gray_blurred_0;

Mat medianBlurring(Mat image);
Mat grayScl(Mat median);
Mat fgbgMOG2(Mat median);
vector<Point2f> cornerDetection(Mat gray_blurred, Mat fgmask);
vector<Point2f> fgbgOpticFlow(Mat gray_blurred, Mat gray_blurred_0, vector<Point2f> tracking_pts_0);
double *time2contact(vector<Point2f> tracking_pts_0, vector<Point2f> tracking_pts, Mat gray_blurred);
void save_times2contact(vector<Point2f> tracking_pts, double *time_vector, double* times2contact, int width, int height);


/* =======================================================================================================================================
 =======================================================================================================================================*/


void image_pipeline(char* img, int width, int height, double* times2contact)
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
    vector<Point2f> new_corners;
    vector<Point2f> tracking_pts;
    double *time_vector;

    // Conver image buffer to Mat
    Mat image(width, height, CV_8UC2, img);
        
    // Blurs the image with a median filter
    median = medianBlurring(image);
    
    // Convert the blurred image to grayscale
    gray_blurred = grayScl(median);

    // Compute the foreground mask
    fgmask = fgbgMOG2(median);
    
    // Compute the optical flow in the blurred, grayscale image
    tracking_pts = fgbgOpticFlow(gray_blurred, gray_blurred_0, tracking_pts_0);
    
    // Time to contact (or any other decision maker)
    time_vector = time2contact(tracking_pts_0, tracking_pts, gray_blurred);

    // Convert the time_vector to a matrix
    save_times2contact(tracking_pts, time_vector, times2contact, width, height);
    
    // Detects the new corners on the image (Shi-Tomasi)
    new_corners = cornerDetection(gray_blurred, fgmask);

    // TODO: IT SHOULDN'T BE NECESSARY (MIGHT BE THOUGH)
    // Swap tracking_pts_0 and gray_blurred_0 with current ones
    //tracking_pts_0.reshape(new_corners.channels,new_corners.rows);
    //gray_blurred_0.reshape(gray_blurred.channels,gray_blurred.rows);

    tracking_pts_0 = new_corners;
    gray_blurred_0 = gray_blurred;

}

// Call this function on the first frame
void image_pipeline_init(char* img, int width, int height)
{
    Mat image(width, height, CV_8UC2, img);

    Mat median = medianBlurring(image);
    gray_blurred_0 = grayScl(median);
    Mat fgmask = fgbgMOG2(median);
    tracking_pts_0 = cornerDetection(gray_blurred_0, fgmask);
}

/* =======================================================================================================================================
 =======================================================================================================================================*/


Mat medianBlurring(Mat image)
{
    // Output
    Mat median;

    // Blur it with a median filter (image == input, median == output, 3rd arg == size of kernel (ODD!!!!))
    medianBlur(image, median, KERNEL_SIZE);
    
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
    //Output
    Mat fgmask;

    Ptr<BackgroundSubtractor> pMOG2;
    
    // Create the background subtractor
    pMOG2 = createBackgroundSubtractorMOG2(HISTORY, VAR_THRESHOLD, detect_shadows); //MOG2 approach
    
    // Create the foreground mask (median == input, fgmask == output)
    pMOG2->apply(median,fgmask);
    
    // Output
    return fgmask;
    
}


/* =======================================================================================================================================
=======================================================================================================================================*/


vector<Point2f> cornerDetection(Mat gray_blurred, Mat fgmask)
{    
    // Output
    vector<Point2f> new_corners;

    // Find corners
    goodFeaturesToTrack(gray_blurred, new_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE, fgmask, BLOCK_SIZE, harris_detector, K_HARRIS);
    
    // Output
    return new_corners;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


vector<Point2f> fgbgOpticFlow(Mat gray_blurred, Mat gray_blurred_0, vector<Point2f> tracking_pts_0)
{
    // Variables
    vector<unsigned char> status;
    vector<int> err;
    
    // Output
    vector<Point2f> tracking_pts;
    
    // Optical flow (Lucas-Kanade)
    calcOpticalFlowPyrLK(gray_blurred_0, gray_blurred, tracking_pts_0, tracking_pts, status, err, winSize, MAX_LEVEL, criteria, FLAGS, MIN_EIG_THRESHOLD);
    
    // Output
    return tracking_pts;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/

double *time2contact(vector<Point2f> tracking_pts_0, vector<Point2f> tracking_pts, Mat gray_blurred)
{
/* This function computes the time to contact given the tracking points (corners) of the previous frame and 
 the points returned by the optical flow calculation for the current frame. 
 
 This is the actual time to contact since it is multiplied by the refresh rate of the function (REFRESH_RATE Hz). If this 
 changes, be aware that in "time[idx2] = distance/velocity * REFRESH_RATE", REFRESH_RATE should be changed to the new refresh rate. 
 */
    
    int width  = gray_blurred.cols;             // width of the image
    int height = gray_blurred.rows;             // height of the image
    double center[2];                           // position of the center of the image
    double position_x, position_y, distance;    // posision components and distance (module of the position)
    double velocity_x, velocity_y, velocity;    // velocity components and module of the velocity
    double time_vector[tracking_pts.size()];                  // Time to contact [s]
       
    // Calculate the position of the center of the image
    center   = {0.5 * width, 0.5 * height};
    
    for (int i = 0; i < tracking_pts_0.size(); ++i)
    {
        position_x = center[0] - tracking_pts_0[i].x;
        position_y = center[1] - tracking_pts_0[i].y;

        distance = hypot(position_x,position_y);

        velocity_x = tracking_pts[i].x - tracking_pts_0[i].x;
        velocity_y = tracking_pts[i].y - tracking_pts_0[i].y;

        velocity = hypot(velocity_x,velocity_y);

        time_vector[i] = (distance/velocity) * REFRESH_RATE;
    }
        
    return time_vector; 
}

void save_times2contact(vector<Point2f> tracking_pts, double *time_vector, double* times2contact, int width, int height)
{
    // All points are -1 but the ones on tracking pts
    memset(times2contact, -1, width*height*sizeof(double));

    for (int i = 0; i < tracking_pts.size(); ++i)
    {
        int x = tracking_pts[i].x;
        int y = tracking_pts[i].y;
        times2contact[width*x+y] = time_vector[i];
    } 
}