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

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H
#include <inttypes.h>
#include "state.h"

//extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;
extern void take_decision_periodic(void);
extern void orange_avoider_init(void);
extern int orange_avoider_periodic(void);
extern uint8_t moveWaypointForward(uint8_t, float);
extern uint8_t moveWaypoint(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern uint8_t chooseEducatedIncrementAvoidance(void);
extern void chooseDecisionBasedOnOrange(void);
extern void chooseDecisionBasedOnCorners(void);

int8_t heading_decision;

#endif

