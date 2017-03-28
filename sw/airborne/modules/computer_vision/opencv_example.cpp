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
#define KERNEL_SIZE 17

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

//CANNY parameters
#define CANNY_CHECK 1
#define THRESHOLD_CANNY_1 100
#define THRESHOLD_CANNY_2 200


#define OPENCV_EXAMPLE_VERBOSE TRUE
#define PRINT(string,...) fprintf(stderr, "[opencv_example->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OPENCV_EXAMPLE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

vector<Point2f> tracking_pts_0;
Mat image_old;

void time2contact(vector<Point2f>& tracking_pts_0, vector<Point2f>& tracking_pts, Mat& gray_blurred, double* time_vector);
void save_times2contact(vector<Point2f>& tracking_pts, double* time_vector, double* times2contact, int width, int height);



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

    printf("Inside image_pipeline \n");
    
    vector<Point2f> tracking_pts;
    vector<uchar> status;
    vector<float> err;
    double *time_vector;    // Time to contact [s]   

    // Conver image buffer to Mat
    Mat M(height, width, CV_8UC2, img);
    Mat image;
    cvtColor(M,image,CV_YUV2BGR_Y422);

    // Compute the optical flow in the blurred, grayscale image
    if (tracking_pts_0.size() == 0)
    {
        fill_array_with_minus_one(times2contact, width*height);
    } else {
        printf("Calculating optical flow\n");
        calcOpticalFlowPyrLK(image_old, image, tracking_pts_0, tracking_pts, status, err, winSize, MAX_LEVEL, criteria, FLAGS, MIN_EIG_THRESHOLD);
        // Time to contact 
        time_vector = (double *)malloc(tracking_pts.size()*sizeof(double));
        time2contact(tracking_pts_0, tracking_pts, image, time_vector);
        printf("Optical flow calculated and timevector saved\n");
    }

    printf("Saving times2contact\n");
    // Convert the time_vector to a matrix
    save_times2contact(tracking_pts, time_vector, times2contact, width, height);
    if (tracking_pts.size())
    {
        free(time_vector);
    }

    Mat canny;
    vector<Point2i> edges;
    int k = 0;
    printf("times2contact saved\n");
    // Edges detection --> Output = canny
    Canny(image, canny, THRESHOLD_CANNY_1, THRESHOLD_CANNY_2);
    printf("Canny done\n");
    // Find the edges in the output
    findNonZero(canny, edges);
    printf("Edges found\n");
    // Allocate the vector of corners
    vector<Point2f> new_corners(edges.size());

    // Get the coordinates of the coordinates
    for (int i = 0; i < canny.cols; ++i)
    {
        for (int j = 0; j < canny.rows; ++j)
        {
            if (canny.at<uchar>(j,i) == 255)    
            {
                new_corners[k].x = (float)i;
                new_corners[k].y = (float)j;
                //printf("new_corners (x) %1.3f \n",new_corners[k].x);
                //printf("new_corners (y) %1.3f \n",new_corners[k].y);
                k++;
            }    
        }
    }
    printf("Saving buffers\n");
    tracking_pts_0.clear();
    tracking_pts_0 = new_corners;
    image_old.release();
    image_old = image;
    printf("Buffers saved\n");
    grayscale_opencv_to_yuv422(canny,img);


// Release memory
    canny.release();
    edges.clear();
    tracking_pts.clear();
//    M.release();
    image.release();
    status.clear();
    err.clear();
}


/* =======================================================================================================================================
 =======================================================================================================================================*/


// Call this function on the first frame
void image_pipeline_init(char* img, int width, int height)
{
    Mat M(height, width, CV_8UC2, img);
    Mat image;

    cvtColor(M,image,CV_YUV2BGR_Y422);

    Mat canny;
    vector<Point2i> edges;

    // Edges detection --> Output = canny
    Canny(image, canny, THRESHOLD_CANNY_1, THRESHOLD_CANNY_2);

    // Find the edges in the output
    findNonZero(canny, edges);

    // Allocate the vector of corners
    vector<Point2f> new_corners(edges.size());

    int k = 0;
    // Get the coordinates of the coordinates
    for (int i = 0; i < canny.cols; ++i)
    {
        for (int j = 0; j < canny.rows; ++j)
        {
            if (canny.at<uchar>(j,i) == 255)    
            {
                new_corners[k].x = i;
                new_corners[k].y = j;
                //printf("new_corners (x) %1.3f \n",new_corners[k].x);
                //printf("new_corners (y) %1.3f \n",new_corners[k].y);
                k++;
            }    
        }
    }

    image_old = image;
    tracking_pts_0 = new_corners;
    canny.release();
    edges.clear();

//   M.release();
     image.release();
}

void time2contact(vector<Point2f>& tracking_pts_old, vector<Point2f>& tracking_pts, Mat& gray_blurred, double* time_vector)
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
    
    // Calculate the position of the center of the image
    center[0] = 0.5 * width;
    center[1] = 0.5 * height;
    
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
            time_vector[i]=-1;
        } else 
        {
            time_vector[i] = (distance/velocity) / REFRESH_RATE;    
        }
    }
}

/* =======================================================================================================================================
 =======================================================================================================================================*/

void save_times2contact(vector<Point2f>& tracking_pts, double *time_vector, double *times2contact, int width, int height)
{
    // All points are -1 but the ones on tracking pts
    fill_array_with_minus_one(times2contact, width*height);   

    mintime2contact_l = 1000.0;
    mintime2contact_c = 1000.0;
    mintime2contact_r = 1000.0; 

    for (unsigned int i = 0; i < tracking_pts.size(); ++i)
    {
        int x = tracking_pts[i].x;
        int y = tracking_pts[i].y;
        times2contact[width*y+x] = time_vector[i];
        //printf("time_vector %1.1f\n",time_vector[i]);
        if (x < width/3 && time_vector[i] < mintime2contact_l)
        {
            mintime2contact_l = time_vector[i];
        }
        else if (x > 2*width/3 && time_vector[i] < mintime2contact_r)
        {
            mintime2contact_r = time_vector[i];
        }
        else if (time_vector[i] < mintime2contact_c)
        {
            mintime2contact_c = time_vector[i];
        }
    }

}

/* =======================================================================================================================================
 =======================================================================================================================================*/

void fill_array_with_minus_one(double *array, int npixels)
{
    for (int i = 0; i < npixels; ++i)
    {
        array[i] = -1.0;
    }
}

