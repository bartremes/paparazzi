/*
 * Copyright (C) 2015  Kirk Scehper <kirkscheper@gmail.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file subsystems/datalink/bluegiga.h
 * bluegiga Bluetooth chip I/O
 */

#ifndef BLUEGIGA_DATA_LINK_H
#define BLUEGIGA_DATA_LINK_H

#include "mcu_periph/link_device.h"
#include "generated/airframe.h"

#define BLUEGIGA_RX_BUFFER_SIZE 20
#define BLUEGIGA_TX_BUFFER_SIZE 20
#define BLUEGIGA_BUFFER_NUM 2

struct bluegiga_periph {
  int curbuf;
  /* Receive buffer */
  volatile uint8_t rx_buf[BLUEGIGA_BUFFER_NUM][BLUEGIGA_RX_BUFFER_SIZE];
  volatile uint16_t rx_insert_idx[BLUEGIGA_BUFFER_NUM];
  /* Transmit buffer */
  volatile uint8_t tx_buf[BLUEGIGA_BUFFER_NUM][BLUEGIGA_TX_BUFFER_SIZE];
  volatile uint16_t tx_insert_idx[BLUEGIGA_BUFFER_NUM];
  // uint8_t tx_running;
  /** Generic device interface */
  struct link_device device;
};

extern uint8_t bluegiga_rx_buf[BLUEGIGA_RX_BUFFER_SIZE];

extern struct bluegiga_periph bluegiga_p;

void bluegiga_init( void );
bool_t bluegiga_check_free_space(int len);
void bluegiga_transmit( uint8_t data );
uint16_t bluegiga_receive( uint8_t *buf, uint16_t len );
void bluegiga_send( void );
//uint16_t bluegiga_rx_size( uint8_t _s );
bool_t bluegiga_ch_available( void );

// Defines that are done in mcu_periph on behalf of uart.
// We need to do these here...
//TODO: check
//#define BlueGigaInit() bluegiga_init()
//#define BlueGigaTxRunning bluegiga_p.tx_running
//#define BlueGigaSetBaudrate(_b) bluegiga_set_baudrate(_b)


// BLUEGIGA is using pprz_transport
// FIXME it should not appear here, this will be fixed with the rx improvements some day...
// BLUEGIGA needs a specific read_buffer function
#include "subsystems/datalink/pprz_transport.h"

static inline void bluegiga_read_buffer( struct pprz_transport *t ) {
//  while ( bluegiga_ch_available() ) {
//    bluegiga_receive( bluegiga_rx_buf, BLUEGIGA_RX_BUFFER_SIZE );
//    int c = 0;
//    do {
//      parse_pprz( t, bluegiga_rx_buf[ c++ ] );
//    } while ( ( t->status != UNINIT ) && !(t->trans_rx.msg_received) );
//  }
  if( bluegiga_ch_available() ) {
    bluegiga_receive( bluegiga_rx_buf, BLUEGIGA_RX_BUFFER_SIZE );
    int c = 0;
    do {
      parse_pprz( t, bluegiga_rx_buf[ c++ ] );
    } while ( ( t->status != UNINIT ) && !(t->trans_rx.msg_received) );
  }
}

// Device interface macros

#define BlueGigaCheckFreeSpace(_x)  (((superbitrf.tx_insert_idx+1) %128) != superbitrf.tx_extract_idx)
#define BlueGigaTransmit(_x) bluegiga_transmit(_x)
#define BlueGigaSendMessage() bluegiga_send()
#define BlueGigaChAvailable() bluegiga_ch_available()
//TODO: check
#define BlueGigaGetch() bluegiga_getch()
// transmit previous date in buffer
#define BlueGigaCheckAndParse(_dev,_trans) {       \
    if (BlueGigaChAvailable()) {                   \
      bluegiga_read_buffer( &(_trans) );           \
      if (_trans.trans_rx.msg_received) {          \
        pprz_parse_payload(&(_trans));             \
        _trans.trans_rx.msg_received = FALSE;      \
      }                                            \
    }                                              \
  }


#endif /* BLUEGIGA_DATA_LINK_H */

