/*
 * Copyright (C) Roland Meertens
 * Edited by Group 10 - AFMAV course
 * March 2016
 * Based on [orange_avoider.c]
 */

#include "modules/group10/variance_avoider.h"
#include "modules/group10/variance_calculator.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>

uint8_t safeToGoForwards=FALSE;
int32_t incrementForAvoidance;

void variance_avoider_init() {

	chooseRandomIncrementAvoidance();
}

void variance_avoider_periodic() {

	safeToGoForwards = safe_fw;
	//printf("Safe_fw: %d\n ret_r: %d\n",safe_fw,ret_r);
}

/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
	  *heading = *heading + increment;
	  // Check if your turn made it go out of bounds
	  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC
	  return FALSE;
}

uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}


//----Choose yaw direction based on variance in image
uint8_t chooseRandomIncrementAvoidance(){

	if(ret_r==1){
		incrementForAvoidance=450;
	}
	else{
		incrementForAvoidance=-450;
	}
	return FALSE;

}
