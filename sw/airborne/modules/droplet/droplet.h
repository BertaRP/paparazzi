/*
 * Copyright (C) Kirk Scheper
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/droplet/droplet.h"
 * @author Kirk Scheper
 * Droplet collision avoidance system
 */

#ifndef DROPLET_H
#define DROPLET_H

#include "stdint.h"

extern void droplet_init(void );
extern void droplet_periodic(void);
extern void run_droplet(uint32_t disparities_total, uint32_t disparities_high);

#endif

