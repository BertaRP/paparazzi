/*
 * OBSTACLE AVOIDER
 * 
 *
 * This file is based on the orange_avoider.c by 
 * Roland Meertens. Modified to take a heading decision
 * based on the time to contact to the detected features
 * and orange and black obstacles (as a double check)
 * 
 * Currently orange and black filters disabled, and thresholds,
 * and RandomIncrementForAvoidance() restored to initial situation.
 * Increment for avoidance based on colors working in branch: OrangeBlackv2_r
 *
 */


#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/navigation.h"

#include "generated/flight_plan.h"
#include "modules/computer_vision/colorfilter.h"
#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/cv_opencvdemo.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

// Trigger to use the orange and black filter as a double check
#define COLOR_AVOIDER_CHECK 0 // (0 ==> color filter disabled, 1 ==> color filter enabled)

// Degrees for the different types of turn
#define SHARP_TURN 20
#define NORMAL_TURN 10


#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);



uint8_t safeToGoForwards        = false;
int thresholdColorCountO        = 0.025 * 260*128; // 520 x 240 = 124.800 total pixels
int thresholdColorCountB        = 0.06 * 124800/3;  // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void orange_avoider_init()
{
   // Initialise the variables of the colorfilter to accept black
  color_lum_minB = 10;
  color_lum_maxB = 18;
  color_cb_minB  = 127;
  color_cb_maxB  = 150;
  color_cr_minB  = 127;
  color_cr_maxB  = 150;
  // Initialise the variables of the colorfilter to accept orange
  color_lum_minO = 20;
  color_lum_maxO = 255;
  color_cb_minO  = 75;
  color_cb_maxO  = 145;
  color_cr_minO  = 167;
  color_cr_maxO  = 255;  
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}

/*
Takes a decision based on the time to contact and the black and orange filters 
(only if COLOR_AVOIDER_CHECK = 1). Calls the corresponding function, which will be 
the one that moves the waypoints.
*/
void take_decision_periodic()
{

#if COLOR_AVOIDER_CHECK
  int no_color = orange_avoider_periodic();
#endif
  heading_decision = corner_avoider_periodic();


#if COLOR_AVOIDER_CHECK
  
  if (no_color)
  {

#endif
  chooseDecisionBasedOnCorners();


#if COLOR_AVOIDER_CHECK
  } else {
      chooseDecisionBasedOnOrange();
  }
#endif
}

/*
Moves the waypoint and modifies the trajectory confidence based on the 
heading decision (based on corners)
*/
void chooseDecisionBasedOnCorners()
{
  float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
  if(heading_decision==0){
      moveWaypointForward(WP_GOAL, moveDistance);
      moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
      nav_set_heading_towards_waypoint(WP_GOAL);
      chooseRandomIncrementAvoidance();
      trajectoryConfidence += 1;
  }
  else{
      chooseEducatedIncrementAvoidance();
      waypoint_set_here_2d(WP_GOAL);
      waypoint_set_here_2d(WP_TRAJECTORY);
      increase_nav_heading(&nav_heading, incrementForAvoidance);
      if(trajectoryConfidence > 5){
          trajectoryConfidence -= 4;
      }
      else{
          trajectoryConfidence = 1;
      }
  }
}

/*
Moves the waypoint and modifies the trajectory confidence based on the 
heading decision (based on orange)
*/
void chooseDecisionBasedOnOrange()
{
  chooseRandomIncrementAvoidance();
  waypoint_set_here_2d(WP_GOAL);
  waypoint_set_here_2d(WP_TRAJECTORY);
  increase_nav_heading(&nav_heading, incrementForAvoidance);
  if(trajectoryConfidence > 5){
      trajectoryConfidence -= 4;
  }
  else{
      trajectoryConfidence = 1;
  }
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
int orange_avoider_periodic()
{
  // Check the amount of orange and black pixels. If this is above a threshold
  // you want to turn a certain amount of degrees 
    int no_color = color_countOc < thresholdColorCountO; // Black threshold currently disabled!

  return no_color;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  //VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + 0.5 * POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + 0.5 * POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)*180/M_PI);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
Choose the increment for avoidance depending on the heading decision
*/
uint8_t chooseEducatedIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  switch (heading_decision)
  {
    case -2: incrementForAvoidance = -SHARP_TURN;
    case -1: incrementForAvoidance = -NORMAL_TURN;
    case 1: incrementForAvoidance = NORMAL_TURN;
    case 2: incrementForAvoidance = SHARP_TURN;
  }
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance()
{

  int r = rand() % 2;
  if (r == 0)
  {
    incrementForAvoidance = NORMAL_TURN;
  }
  else 
  {
    incrementForAvoidance = -NORMAL_TURN;
  }

  return false;
}



