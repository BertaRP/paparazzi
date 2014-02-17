/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/lpc21/subsystems/actuators/servos_ppm_hw.h
 *
 * Efficient driving of MAT0.1 (SERVO_CLOCK_PIN) using TIMER0 to produce PPM
 * for a R/C receiver which has a microcontroller to drive the servos
 * (not a 4015 or 4017 decade counter chip).
 */

#ifndef SERVOS_PPM_HW_H
#define SERVOS_PPM_HW_H

#include "std.h"
#include "LPC21xx.h"
#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG

#define SERVOS_TICS_OF_USEC(s) CPU_TICKS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#define _PPM_NB_CHANNELS 5
extern uint16_t servos_values[_PPM_NB_CHANNELS];
#define ActuatorPpmSet(_i, _v) { servos_values[_i] = SERVOS_TICS_OF_USEC(_v); }

extern void actuators_ppm_init(void);
#define ActuatorsPpmCommit() {}
#define ActuatorsPpmInit() actuators_ppm_init()
extern uint8_t servos_PPM_idx;
extern uint32_t servos_delay;
extern uint8_t ppm_pulse; /* 1=start of pulse, 0=end of pulse */

/* define the ppm frame rate. 22msec is typical for JR/Graupner */
#define SERVO_REFRESH_TICS SERVOS_TICS_OF_USEC(16100)
/* define the ppm pulse width. 550usec is typical for JR/Graupner */
#define PPM_WIDTH SERVOS_TICS_OF_USEC(400)

#define ACTUATORS_IT TIR_MR1I
#define ServosPPMMat_ISR() {				\
  if (servos_PPM_idx == 0) {				\
    SetBit(IO1CLR, SERVO_RESET_PIN);				\
    servos_delay = SERVO_REFRESH_TICS;			\
  }							\
  if (servos_PPM_idx < _PPM_NB_CHANNELS ) {		\
    if (ppm_pulse) { 					\
	T0MR1 += PPM_WIDTH;				\
        servos_delay -= PPM_WIDTH; 			\
    } else {						\
	T0MR1 += servos_values[servos_PPM_idx] - PPM_WIDTH;		\
    	servos_delay -= servos_values[servos_PPM_idx] - PPM_WIDTH;	\
    	servos_PPM_idx++;					\
    }                                                   \
  } else { /* servos_PPM_idx=_PPM_NB_CHANNELS */ \
    SetBit(IO1SET, SERVO_RESET_PIN);				\
    if (ppm_pulse) { 					\
	T0MR1 += PPM_WIDTH;				\
        servos_delay -= PPM_WIDTH; 			\
    } else {						\
    servos_PPM_idx = 0;				\
    T0MR1 += servos_delay - PPM_WIDTH;				\
    }                                                   \
  }							\
  if (!ppm_pulse) {                                     \
  /* lower clock pin */					\
  T0EMR &= ~TEMR_EM1;                                   \
  }					                \
  /* toggle ppm_pulse flag */				\
  ppm_pulse ^= 1;                                       \
}
#endif /* SERVOS_PPM_HW_H */