/* IMG_PROCESSING
 
 This module blablabla finish this introduction
 
 */



#include "opencv_example.h"
#include <opencv2/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d.hpp>
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
#define QUALITY_LEVEL 0.0001
#define MIN_DISTANCE 0.1
#define BLOCK_SIZE 7
static const bool harris_detector = false;
#define K_HARRIS 0.04

// Optic Flow parameters
#define MAX_LEVEL 3
#define FLAGS 0
#define MIN_EIG_THRESHOLD 0.0001
static const TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01); // Termination criteria (iterations, delta)
static const Size winSize = Size(10,10);

// FAST parameters
#define THRESHOLD_FAST 50
static const bool nonmax_suppresion = false;


#define OPENCV_EXAMPLE_VERBOSE TRUE
#define PRINT(string,...) fprintf(stderr, "[opencv_example->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OPENCV_EXAMPLE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

vector<Point2f> tracking_pts_0;
Mat gray_blurred_0;

Mat medianBlurring(Mat& image);
Mat grayScl(Mat& median);
Mat fgbgMOG2(Mat& median);
vector<Point2f> cornerDetection(Mat& gray_blurred, Mat& fgmask);
vector<Point2f> fgbgOpticFlow(Mat& gray_blurred, Mat& gray_blurred_0, vector<Point2f>& tracking_pts_0);
double *time2contact(vector<Point2f>& tracking_pts_0, vector<Point2f>& tracking_pts, Mat& gray_blurred);
void save_times2contact(vector<Point2f>& tracking_pts, double *time_vector, double* times2contact, int width, int height);
vector<KeyPoint> FAST_detect(Mat& gray_blurred);
void keypoints_in_image(Mat& gray_blurred, vector<KeyPoint> keypoints_fast);
vector<Point2f> keypoints_to_vector(vector<KeyPoint> keypoints_fast);


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
    //printf("image_pipeline!!!\n");
    Mat median;
    Mat gray_blurred;
    Mat fgmask;
    vector<Point2f> new_corners;
    vector<Point2f> tracking_pts;
    vector<KeyPoint> keypoints_fast;
    double *time_vector;

    // Conver image buffer to Mat
    Mat M(height, width, CV_8UC2, img);
    Mat image;
    cvtColor(M,image,CV_YUV2BGR_Y422);

    // Blurs the image with a median filter
   // median = medianBlurring(image);
    //printf("Median\n");
    // Convert the blurred image to grayscale
    gray_blurred = grayScl(image);
    //printf("gray\n");
    // Compute the foreground mask
    //fgmask = fgbgMOG2(median);
    //printf("fgmask\n");
    // Compute the optical flow in the blurred, grayscale image
    //printf("tracking_pts_0 size (%1.3f,%1.3f) \n",tracking_pts_0[0].x, tracking_pts_0[0].y);
    //printf("tracking_pts_0 size (%1.3f,%1.3f) \n",tracking_pts_0[1].x, tracking_pts_0[1].y);

    if (tracking_pts_0.size() == 0)
    {
        fill_array_with_minus_one(times2contact, width*height);
    } else {
        tracking_pts = fgbgOpticFlow(gray_blurred, gray_blurred_0, tracking_pts_0);
        // Time to contact (or any other decision maker)
        time_vector = time2contact(tracking_pts_0, tracking_pts, gray_blurred);
    }
    // Convert the time_vector to a matrix
    save_times2contact(tracking_pts, time_vector, times2contact, width, height);

    // Detects the new corners on the image (Shi-Tomasi) // FAST algorithm
    //printf("fgmask size:%d \n",fgmask.size());
    //new_corners = cornerDetection(gray_blurred, fgmask);
    keypoints_fast = FAST_detect(gray_blurred);

    // Convert the keypoints to an vector
    new_corners = keypoints_to_vector(keypoints_fast);

    // Set the current corners and image as "old" values for next frame
    tracking_pts_0 = new_corners;
    gray_blurred_0 = gray_blurred;

    // Dray the keypoints in the image
    //keypoints_in_image(image, keypoints_fast);

    // Set the image with the keypoints to be shown in camera
   // colorrgb_opencv_to_yuv422(image, img);
}


// Call this function on the first frame
void image_pipeline_init(char* img, int width, int height)
{
    //printf("Para el puto alber\n");
    Mat M(height, width, CV_8UC2, img);
    Mat image;
    vector<KeyPoint> keypoints_0;

    cvtColor(M,image,CV_YUV2BGR_Y422);
    //Mat median = medianBlurring(image);
    gray_blurred_0 = grayScl(image);
    //Mat fgmask = fgbgMOG2(median);
    //tracking_pts_0 = cornerDetection(gray_blurred_0, fgmask);
    keypoints_0 = FAST_detect(gray_blurred_0);
    tracking_pts_0 = keypoints_to_vector(keypoints_0);


}

/* =======================================================================================================================================
 =======================================================================================================================================*/


Mat medianBlurring(Mat& image)
{
    // Output
    Mat median;

    // Blur it with a median filter (image == input, median == output, 3rd arg == size of kernel (ODD!!!!))
    medianBlur(image, median, KERNEL_SIZE);
    //blur(image, median, Size(7,7));
    
    // Output
    return median;
}

/* =======================================================================================================================================
 =======================================================================================================================================*/

Mat grayScl(Mat& median)
{
    // Output
    Mat gray_blurred;
    
    // Convert the image (the median) to grayscale
    cvtColor(median, gray_blurred, CV_BGR2GRAY);    

    return gray_blurred;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


Mat fgbgMOG2(Mat& median)
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


vector<Point2f> cornerDetection(Mat& gray_blurred, Mat& fgmask)
{    
    // Output
    vector<Point2f> new_corners;
    //Mat mask(gray_blurred.size(),CV_8UC1);
    //mask.setTo(Scalar::all(1));
    
    // Find corners
    goodFeaturesToTrack(gray_blurred, new_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE, Mat(), BLOCK_SIZE, harris_detector, K_HARRIS);
        
    // Output
    return new_corners;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


vector<Point2f> fgbgOpticFlow(Mat& gray_blurred, Mat& gray_blurred_old, vector<Point2f>& tracking_pts_old)
{
    // Variables
    vector<uchar> status;
    vector<float> err;
    
    // Output
    vector<Point2f> tracking_pts;
    
    // Optical flow (Lucas-Kanade)
    calcOpticalFlowPyrLK(gray_blurred_old, gray_blurred, tracking_pts_old, tracking_pts, status, err, winSize, MAX_LEVEL, criteria, FLAGS, MIN_EIG_THRESHOLD);

    // Output
    return tracking_pts;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/

vector<KeyPoint> FAST_detect(Mat& gray_blurred)
{
    vector<KeyPoint> keypoints_fast;

    FAST(gray_blurred, keypoints_fast, THRESHOLD_FAST, nonmax_suppresion);

    return keypoints_fast;
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


double *time2contact(vector<Point2f>& tracking_pts_old, vector<Point2f>& tracking_pts, Mat& gray_blurred)
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
    double time_vector[tracking_pts.size()];    // Time to contact [s]
       
    // Calculate the position of the center of the image
    center[0] = 0.5 * width;
    center[1] = 0.5 * height;

    min_time2contact = 1000.0;
    
    for (unsigned int i = 0; i < tracking_pts.size(); ++i)
    {
        position_x = center[0] - tracking_pts_old[i].x;
        position_y = center[1] - tracking_pts_old[i].y;

        distance = sqrt(position_x*position_x+position_y*position_y);

        velocity_x = tracking_pts[i].x - tracking_pts_old[i].x;
        velocity_y = tracking_pts[i].y - tracking_pts_old[i].y;

        velocity = sqrt(velocity_x*velocity_x+velocity_y*velocity_y);

        if (velocity == 0)
        {
            time_vector[i] = -1;
        } 
        else 
        {
            time_vector[i] = (distance/velocity) / REFRESH_RATE;
        
            if (time_vector[i] < min_time2contact)
            {
                min_time2contact = time_vector[i];
            }       
        }
    }
        
    return time_vector; 
}

void save_times2contact(vector<Point2f>& tracking_pts, double *time_vector, double *times2contact, int width, int height)
{
    // All points are -1 but the ones on tracking pts
    fill_array_with_minus_one(times2contact, width*height);    

    for (unsigned int i = 0; i < tracking_pts.size(); ++i)
    {
        int x = tracking_pts[i].x;
        int y = tracking_pts[i].y;
        times2contact[width*y+x] = time_vector[i];
    } 
}

void fill_array_with_minus_one(double *array, int npixels)
{
    for (int i = 0; i < npixels; ++i)
    {
        array[i] = -1.0;
    }
}


void keypoints_in_image(Mat& gray_blurred, vector<KeyPoint> keypoints_fast)
{
    drawKeypoints(gray_blurred, keypoints_fast, gray_blurred, Scalar::all(0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}


vector<Point2f> keypoints_to_vector(vector<KeyPoint> keypoints_fast)
{

    vector<Point2f> keypoints;
    vector<int> keypoints_indexes;


    keypoints_indexes.reserve(keypoints.size());
    for (int i = 0; i < keypoints.size(); ++i)
    {
        keypoints_indexes.push_back(i);
    }

    KeyPoint::convert(keypoints_fast, keypoints, keypoints_indexes);    

    return keypoints;

}