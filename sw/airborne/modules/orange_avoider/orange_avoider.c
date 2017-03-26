/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
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
#define ORANGE_AVOIDER_CHECK 0 // Toggle check of orange avoider before corner_avoider
#define SHARP_TURN 30
#define NORMAL_TURN 10


#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);



uint8_t safeToGoForwards        = false;
int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void orange_avoider_init()
{
  // Initialise the variables of the colorfilter to accept orange
  color_lum_min = 20;
  color_lum_max = 255;
  color_cb_min  = 75;
  color_cb_max  = 145;
  color_cr_min  = 167;
  color_cr_max  = 255;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}


void take_decision_periodic()
{

#if ORANGE_AVOIDER_CHECK
  int no_orange = orange_avoider_periodic();
#endif
  heading_decision = corner_avoider_periodic();
  //VERBOSE_PRINT("Heading_decision: %d \n", heading_decision);


#if ORANGE_AVOIDER_CHECK
  
  if (no_orange)
  {

#endif
    chooseDecisionBasedOnCorners(heading_decision);

#if ORANGE_AVOIDER_CHECK
  } else {
      chooseDecisionBasedOnOrange();
  }
#endif
}


void chooseDecisionBasedOnCorners(int heading_decision)
{
  //VERBOSE_PRINT("Entering: chooseDecisionBasedOnCorners: %d \n", 0);
  float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
  if(heading_decision==0){
      //VERBOSE_PRINT("chooseDecisionBasedOnCorners --> heading_decision = 0: %d \n", 0);
      moveWaypointForward(WP_GOAL, moveDistance);
      moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
      nav_set_heading_towards_waypoint(WP_GOAL);
      chooseRandomIncrementAvoidance();
      trajectoryConfidence += 1;
  }
  else{
      //VERBOSE_PRINT("chooseDecisionBasedOnCorners --> heading_decision != 0: %d \n", 0);
      chooseEducatedIncrementAvoidance(heading_decision);
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
  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  int no_orange = color_count < tresholdColorCount;
  //VERBOSE_PRINT("Color_count: %d  threshold: %d no_orange: %d \n", color_count, tresholdColorCount, no_orange);
  return no_orange;
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
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
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

uint8_t chooseEducatedIncrementAvoidance(int heading_decision)
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
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = NORMAL_TURN;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -NORMAL_TURN;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}

