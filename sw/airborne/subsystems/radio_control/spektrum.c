/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *               2010 Eric Parsonage <eric@eparsonage.com>
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "spektrum.h"

#include "std.h"
#include "subsystems/radio_control.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time.h"

// for timer_get_frequency
#include "mcu_arch.h"
INFO("Radio-Control now follows PPRZ sign convention: this means you might need to reverese some channels in your transmitter: RollRight / PitchUp / YawRight / FullThrottle / Auto2 are positive deflections")

/* Automatically determine amount of satellites */
#if defined SPEKTRUM_UART_SAT4
#define SPEKTRUM_SATELLITES_NB 4
#elif defined SPEKTRUM_UART_SAT3
#define SPEKTRUM_SATELLITES_NB 3
#elif defined SPEKTRUM_UART_SAT2
#define SPEKTRUM_SATELLITES_NB 2
#elif defined SPEKTRUM_UART_SAT1
#define SPEKTRUM_SATELLITES_NB 1
#else
#error "You must at least define 1 Spektrum satellite receiver. (SPEKTRUM_UART_SAT1 must be defined)"
#endif

/* Timings and maximums of Spektrum DSM protocol */
#define SPEKTRUM_CHANNELS_PER_FRAME 7   ///< Maximum amount of RC channels per frame
#define SPEKTRUM_MAX_FRAMES 2           ///< Maximum amount of RC frames containing different channels
#define SPEKTRUM_MAX_CHANNELS (SPEKTRUM_CHANNELS_PER_FRAME * SPEKTRUM_MAX_FRAMES)
#define SPEKTRUM_GUESS_FRAMES 6         ///< Amount of frames needed for guessing TX type
#define SPEKTRUM_MIN_FRAME_SPACE  7     ///< Minum amount of time between frames (7ms)
#define SPEKTRUM_MAX_FRAME_TIME  4      ///< Maximum amount a frame takes to receive (4ms)

/* Number of low pulses sent during binding to the satellite receivers */
#ifndef SPEKTRUM_BIND_PULSES
#define SPEKTRUM_BIND_PULSES 10
#endif

/* Set polarity using RC_POLARITY_GPIO. */
#ifndef RC_SET_POLARITY
#define RC_SET_POLARITY gpio_clear
#endif

/* Per satellite we keep track of data */
struct spektrum_sat_t {
  bool valid;                 ///< True when we received a packet else false
  uint32_t timer;             ///< Timer to keep track of the UART synchronisation
  struct uart_periph *dev;    ///< UART device which the satellite is connected to
  uint8_t lost_frame_cnt;     ///< Amount of RC frames lost
  uint8_t tx_type;            ///< Transmitter type encoded (see wiki)
  int16_t values[SPEKTRUM_MAX_CHANNELS];  ///< RC channel values
};

/* Main spektrum structure */
struct spektrum_t {
  bool valid;                               ///< True when we received a packet else false
  int8_t signs[RADIO_CONTROL_NB_CHANNEL];   ///< Signs for the RC channels
  uint8_t bind_pulses;                      ///< Amount of bind pulses
  struct spektrum_sat_t satellites[SPEKTRUM_SATELLITES_NB]; ///< All the satellites connected
};
static struct spektrum_t spektrum = {
  .valid = false,
  .signs = RADIO_CONTROL_SPEKTRUM_SIGNS,
  .bind_pulses = SPEKTRUM_BIND_PULSES
};

/* Channel masks for detection */
static int16_t spektrum_det_masks[][2] = {
  {0x003F, 0x003F}, /* 6 channels (DX6) */
  {0x007F, 0x007F}, /* 7 channels (DX7) */
  {0x007F, 0x0080}, /* 8 channels (DX8) */
  {0x007F, 0x0180}, /* 9 channels (DX9) */
  {0x007F, 0x0380}, /* 10 channels (DX10) */
  {0x007F, 0x1F80}, /* 13 channels (DX10t) */
  {0x007F, 0x7F80}, /* 14 channels (DX10) */
};

static void spektrum_try_bind(void);
static void spektrum_uart_got_byte(struct uart_periph *dev);

/** Initialize a spektrum sattelite */
static inline void spektrum_init_sat(struct spektrum_sat_t *sat, struct uart_periph *dev)
{
  sat->valid = false;
  sat->tx_type = 0;
  sat->dev = dev;
  sat->dev->recv_cb = &spektrum_uart_got_byte;

  // Initialize values
  for(uint8_t i = 0; i < SPEKTRUM_MAX_CHANNELS; i++) {
    sat->values[i] = 0;
  }
}

/** Main Radio initialization */
void radio_control_impl_init(void)
{
  // Set polarity to normal on boards that can change this
#ifdef RC_POLARITY_GPIO_PORT
  gpio_setup_output(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
  RC_SET_POLARITY(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
#endif

  // Initialize all the UART's in the satellites (TODO: fix nice callback)
#if defined SPEKTRUM_UART_SAT1
  spektrum_init_sat(&spektrum.satellites[0], &SPEKTRUM_UART_SAT1);
#endif
#if defined SPEKTRUM_UART_SAT2
  spektrum_init_sat(&spektrum.satellites[1], &SPEKTRUM_UART_SAT2);
#endif
#if defined SPEKTRUM_UART_SAT3
  spektrum_init_sat(&spektrum.satellites[2], &SPEKTRUM_UART_SAT3);
#endif
#if defined SPEKTRUM_UART_SAT4
  spektrum_init_sat(&spektrum.satellites[3], &SPEKTRUM_UART_SAT4);
#endif

  // Check if the satellites need to be put into binding mode
  spektrum_try_bind();
}

/* Guess TX type */
static inline void spektrum_guess_type(struct spektrum_sat_t *sat, uint16_t chan)
{
  // Use values for parsing
  if(sat->values[0] >= (SPEKTRUM_CHANNELS_PER_FRAME*SPEKTRUM_GUESS_FRAMES))
  {
    // Check all masks which fits
    uint8_t is_10bit = 0;
    uint8_t is_11bit = 0;
    bool is_2packet = false;
    for(uint8_t i = 0; i < (sizeof(spektrum_det_masks) / (sizeof(int16_t)*2)); i++) {
      // Is a 10 bit packet
      if(spektrum_det_masks[i][0] == sat->values[1] && spektrum_det_masks[i][1] == sat->values[2]) {
        is_10bit++;
        is_2packet = (sat->values[1] != sat->values[2]);
      }
      if(spektrum_det_masks[i][0] == sat->values[3] && spektrum_det_masks[i][1] == sat->values[4]) {
        is_11bit++;
        is_2packet = (sat->values[3] != sat->values[4]);
      }
    }

    // Reset everything
    sat->values[0] = 0;
    sat->values[1] = 0;
    sat->values[2] = 0;
    sat->values[3] = 0;
    sat->values[4] = 0;

    // Check if valid and set tx type
    if(is_10bit == 1 && is_11bit == 0)
      sat->tx_type = is_2packet ? 0x2 : 0x1;
    else if(is_10bit == 0 && is_11bit == 1)
      sat->tx_type = is_2packet ? 0x12 : 0x11;
    else
      sat->tx_type = 0x11;
    return;
  }

  // If the channel is not set
  if(chan == 0xFFFF) {
    sat->values[0]++;
    return;
  }

  // We received the second frame
  if((sat->values[0] % SPEKTRUM_CHANNELS_PER_FRAME) == 0 && chan & 0x8000) {
    sat->values[2] |= 1 << ((chan >> 10) & 0xF);
    sat->values[4] |= 1 << ((chan >> 11) & 0xF);
  }
  // We received the first frame
  else {
    sat->values[1] |= 1 << ((chan >> 10) & 0xF);
    sat->values[3] |= 1 << ((chan >> 11) & 0xF);
  }

  sat->values[0]++;
}

/* Parse a sattelite channel */
static inline void spektrum_parse_channel(struct spektrum_sat_t *sat, uint16_t chan)
{
  // This channel is not used
  if(chan == 0xFFFF)
    return;

  // If we expect one frame or when this is the second frame
  //if(((sat->tx_type & 0x3) == 1) || (i == 0 && chan & 0x8000)) {
    sat->valid = true;
    spektrum.valid = true;
  //}

  // We got a 11bit precision packet
  if(sat->tx_type & 0x10) {
    uint8_t chan_num = (chan >> 11) & 0xF;
    sat->values[chan_num] = chan & 0x7FF;
    sat->values[chan_num] -= 0x400;
    sat->values[chan_num] *= MAX_PPRZ / 0x2AC;
  }
  // We got a 10bit precision packet
  else {
    uint8_t chan_num = (chan >> 10) & 0xF;
    sat->values[chan_num] = chan & 0x3FF;
    sat->values[chan_num] -= 0x200;
    sat->values[chan_num] *= MAX_PPRZ / 0x156;
  }
}

/* Spektrum parser for a satellite */
static inline void spektrum_parser(struct spektrum_sat_t *sat)
{
  uint8_t i;
  struct link_device *dev = &sat->dev->device; // The link device of the satellite
  uint16_t bytes_cnt = dev->char_available(dev->periph); // The amount of bytes in the buffer
  uint32_t timer = get_sys_time_msec() - sat->timer; // Timer difference

  // Ignore the first byte ever since we don know timing
  if(sat->timer == 0) {
    // Reset the timer
    sat->timer = get_sys_time_msec();
    return;
  }
  // Smaller than min frame space and first byte (Not the first byte of the packet)
  else if(timer < SPEKTRUM_MIN_FRAME_SPACE && bytes_cnt == 1) {
    dev->get_byte(dev->periph); // Clear the buffer and continue searching

    // Reset the timer
    sat->timer = get_sys_time_msec();
    return;
  }
  // Got a valid frame SYNC
  else if(timer >= SPEKTRUM_MIN_FRAME_SPACE && bytes_cnt == 1) {
    // Reset the timer
    sat->timer = get_sys_time_msec();
    return;
  }
  // Timeout and already had a first byte (Didn't receive 16 bytes in time)
  else if(timer > SPEKTRUM_MAX_FRAME_TIME && bytes_cnt > 1) {
    // Clear the total buffer
    for(i = 0; i < bytes_cnt; i++) {
      dev->get_byte(dev->periph);
    }

    // Reset the timer
    sat->timer = get_sys_time_msec();
    return;
  }

  // We got a full packet (16 bytes)
  if(bytes_cnt >= 16) {
    // Parse packet
    sat->lost_frame_cnt = dev->get_byte(dev->periph);
    dev->get_byte(dev->periph); // For now ignore the second byte (which could be the TX type)

    for(i = 0; i < SPEKTRUM_CHANNELS_PER_FRAME; i++) {
      // Get the byte to parse
      uint16_t chan = (dev->get_byte(dev->periph) << 8) | dev->get_byte(dev->periph);

      // If we don't know the TX type yet try to guess it
      if(sat->tx_type == 0)
        spektrum_guess_type(sat, chan);
      else
        spektrum_parse_channel(sat, chan);
    }

    // Reset the timer
    sat->timer = get_sys_time_msec();
  }
}

/** Interrupt when we got a byte on the UART */
static void spektrum_uart_got_byte(struct uart_periph *dev)
{
  uint8_t i;

  // Go trough the satellites and find the correct one
  for(i = 0; i < SPEKTRUM_SATELLITES_NB; i++) {
    if(spektrum.satellites[i].dev == dev) {
      spektrum_parser(&spektrum.satellites[i]);
      break;
    }
  }
}

/** Checks if there is one valid satellite and sets the radio_control structure */
void RadioControlEventImp(void (*frame_handler)(void))
{
  // Whenever we received a valid RC packet
  if(spektrum.valid) {
    uint8_t sat_id = SPEKTRUM_SATELLITES_NB;
    spektrum.valid = false;

    // Find the first satellite that has a valid packet
    for(uint8_t i = 0; i < SPEKTRUM_SATELLITES_NB; i++) {
      if(i < sat_id)
        sat_id = i;
      if(spektrum.satellites[i].valid)
        spektrum.satellites[i].valid = false;
    }

    // Failsafe case if found satellite is out of bound (Should not happen)
    if(sat_id >= SPEKTRUM_SATELLITES_NB)
      return;

    // Set the radio control status
    radio_control.frame_cpt++;
    radio_control.time_since_last_frame = 0;
    radio_control.status = RC_OK;

    // Copy the radio control channels
    for(uint8_t i = 0; i < RADIO_CONTROL_NB_CHANNEL; i++) {
      radio_control.values[i] = spektrum.satellites[sat_id].values[i] * spektrum.signs[i];
      Bound(radio_control.values[i], -MAX_PPRZ, MAX_PPRZ);

      // The throttle is only positive
      if (i == RADIO_THROTTLE) {
        radio_control.values[i] += MAX_PPRZ;
        radio_control.values[i] /= 2;
      }
    }

    // We got a valid frame so execute the frame handler
    (*frame_handler)();
  }
}

/** Transmit the binding pulses on a port */
static inline void spektrum_send_bind_pulses(uint32_t port, uint16_t gpios)
{
  for (uint8_t i = 0; i < SPEKTRUM_BIND_PULSES; i++) {
    gpio_clear(port, gpios);
    sys_time_usleep(120);
    gpio_set(port, gpios);
    sys_time_usleep(120);
  }
}

#define _UART_RX_PORT(i) i ## _GPIO_PORT_RX
#define UART_RX_PORT(i) _UART_RX_PORT(i)
#define _UART_RX(i) i ## _GPIO_RX
#define UART_RX(i) _UART_RX(i)
#define _UART_AF(i) i ## _GPIO_AF
#define UART_AF(i) _UART_AF(i)

/** This function puts the satellite in binding mode.
 * The requirement of this are that this needs to be done while powering up.
 */
static void spektrum_try_bind(void) {
#ifdef SPEKTRUM_BIND_PIN_PORT
#ifdef SPEKTRUM_BIND_PIN_HIGH
  /* Init GPIO for the bind pin, we enable the pulldown resistor. */
  gpio_setup_input_pulldown(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN);

  /* Exit if the BIND_PIN is low, it needs to
     be pulled high at startup to initiate bind */
  if (gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) == 0) {
    return;
  }
#else
  /* Init GPIO for the bind pin, we enable the pullup resistor. */
  gpio_setup_input_pullup(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN);

  /* Exit if the BIND_PIN is high, it needs to
     be pulled low at startup to initiate bind */
  if (gpio_get(SPEKTRUM_BIND_PIN_PORT, SPEKTRUM_BIND_PIN) != 0) {
    return;
  }
#endif
#endif

  /* Set to push-pull and drive the like high */
#if defined SPEKTRUM_UART_SAT1
  gpio_setup_output(UART_RX_PORT(SPEKTRUM_UART_SAT1_UPPER), UART_RX(SPEKTRUM_UART_SAT1_UPPER));
  gpio_set(UART_RX_PORT(SPEKTRUM_UART_SAT1_UPPER), UART_RX(SPEKTRUM_UART_SAT1_UPPER));
#endif

  /* We have no idea how long the window for allowing binding after
     power up is. This works for the moment but will need revisiting */
  sys_time_usleep(61000);

#if defined SPEKTRUM_UART_SAT1
  spektrum_send_bind_pulses(UART_RX_PORT(SPEKTRUM_UART_SAT1_UPPER), UART_RX(SPEKTRUM_UART_SAT1_UPPER));
  gpio_setup_pin_af(UART_RX_PORT(SPEKTRUM_UART_SAT1_UPPER), UART_RX(SPEKTRUM_UART_SAT1_UPPER), UART_AF(SPEKTRUM_UART_SAT1_UPPER), FALSE);
#endif
}
