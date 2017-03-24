/*
 * Copyright (C) Roland Meertens
 * Edited by Group 10 AFMAV Course
 * March 2016
 */

#ifndef VARIANCE_AVOIDER_H
#define VARIANCE_AVOIDER_H
#include <inttypes.h>

extern uint8_t safeToGoForwards;
extern int32_t incrementForAvoidance;
extern void variance_avoider_init(void);
extern void variance_avoider_periodic(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);

#endif
