/*
 * Copyright (C) 2012 Gerard Toonstra
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
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file subsystems/datalink/BLUEGIGA.c
 * datalink implementation for the BlueGiga Bleutooth radio chip trough SPI
 */

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/bluegiga.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"

#define TXBUF_BASE 0x4000
#define RXBUF_BASE 0x6000

#ifndef BLUEGIGA_SPI_DEV
#define BLUEGIGA_SPI_DEV spi2
#endif

#ifndef BLUEGIGA_SLAVE_IDX
#define BLUEGIGA_SLAVE_IDX SPI_SLAVE2
#endif

#ifndef BLUEGIGA_DRDY_GPIO
#define BLUEGIGA_DRDY_GPIO GPIOC
#endif

#ifndef BLUEGIGA_DRDY_GPIO_PIN
#define BLUEGIGA_DRDY_GPIO_PIN GPIO6
#endif

struct bluegiga_periph chip0;
uint8_t ck_a, ck_b;
uint8_t bluegiga_rx_buf[BLUEGIGA_RX_BUFFER_SIZE];

static const uint8_t RST = 7; // Reset BIT

uint16_t SBASE[SOCKETS]; // Tx buffer base address
uint16_t RBASE[SOCKETS]; // Rx buffer base address
static const uint16_t SSIZE = 2048; // Max Tx buffer size
static const uint16_t RSIZE = 2048; // Max Rx buffer size

struct spi_transaction bluegiga_spi;

static void bluegiga_read_data( uint8_t s, volatile uint8_t *src, volatile uint8_t *dst, uint16_t len );
static uint16_t bluegiga_read(uint16_t _addr, uint8_t *_buf, uint16_t _len);

static inline void bluegiga_set(uint16_t _reg, uint8_t _val)
{
  bluegiga_spi.output_buf[0] = 0xF0;
  bluegiga_spi.output_buf[1] = _reg >> 8;
  bluegiga_spi.output_buf[2] = _reg & 0xFF;
  bluegiga_spi.output_buf[3] = _val;

  spi_submit( &(BLUEGIGA_SPI_DEV), &bluegiga_spi );

  // FIXME: no busy waiting! if really needed add a timeout!!!!
  while(bluegiga_spi.status != SPITransSuccess);
}

static inline uint8_t bluegiga_get(uint16_t _reg)
{
  bluegiga_spi.output_buf[0] = 0x0F;
  bluegiga_spi.output_buf[1] = _reg >> 8;
  bluegiga_spi.output_buf[2] = _reg & 0xFF;

  spi_submit( &(BLUEGIGA_SPI_DEV), &bluegiga_spi );

  // FIXME: no busy waiting! if really needed add a timeout!!!!
  while(bluegiga_spi.status != SPITransSuccess);

  return bluegiga_spi.input_buf[3];
}

static inline void bluegiga_set_buffer(uint16_t _reg, volatile uint8_t *_buf, uint16_t _len)
{
  for (int i=0; i<_len; i++) {
      bluegiga_set( _reg, _buf[ i ] );
    _reg++;
  }
}

// Functions for the generic device API
static int true_function(struct bluegiga_periph* p __attribute__((unused)), uint8_t len __attribute__((unused))) { return TRUE; }
static void dev_transmit(struct bluegiga_periph* p __attribute__((unused)), uint8_t byte) {  bluegiga_transmit(byte); }
static void dev_send(struct bluegiga_periph* p __attribute__((unused))) { bluegiga_send(); }

void BLUEGIGA_init( void ) {

  // configure the SPI bus.
  bluegiga_spi.slave_idx = BLUEGIGA_SLAVE_IDX;
  bluegiga_spi.output_length = 20;
  bluegiga_spi.input_length = 20;
  bluegiga_spi.select = SPISelectUnselect;

  bluegiga_spi.cpol = SPICpolIdleHigh;
  bluegiga_spi.cpha = SPICphaEdge1;
  bluegiga_spi.dss = SPIDss8bit;
  bluegiga_spi.bitorder = SPIMSBFirst;
  bluegiga_spi.cdiv = SPIDiv64;

  chip0.status = BLUEGIGAStatusUninit;
  chip0.curbuf = 0;
  bluegiga_spi.input_buf = &chip0.work_rx[0];
  bluegiga_spi.output_buf = &chip0.work_tx[0];

  // wait one second for proper initialization (chip getting powered up).
  sys_time_usleep(1000000);

  // set DRDY pin
  gpio_setup_output(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
  gpio_clear(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
  sys_time_usleep(200);
  gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

  // allow some time for the chip to wake up.
  sys_time_usleep(20000);

  // write reset bit into mode register
  bluegiga_set( REG_MR, 1<<RST );

  // allow some time to wake up...
  sys_time_usleep(20000);

  // receive memory size
  bluegiga_set( REG_RX_MEM, 0x55 );

  // transmit memory size
  bluegiga_set( REG_TX_MEM, 0x55 );

  // Configure generic device
  chip0.device.periph = (void *)(&chip0);
  chip0.device.check_free_space = (check_free_space_t) true_function;
  chip0.device.transmit = (transmit_t) dev_transmit;
  chip0.device.send_message = (send_message_t) dev_send;
}

void bluegiga_transmit( uint8_t data ) {

  uint16_t temp = (chip0.tx_insert_idx[ chip0.curbuf ] + 1) % BLUEGIGA_TX_BUFFER_SIZE;

  if (temp == chip0.tx_extract_idx[ chip0.curbuf ]) {
    // no more room in this transaction.
    return;
  }

  // check if in process of sending data
  chip0.tx_buf[ chip0.curbuf ][ chip0.tx_insert_idx[ chip0.curbuf ] ] = data;
  chip0.tx_insert_idx[ chip0.curbuf ] = temp;
}

void bluegiga_send() {
  // Now send off spi transaction.
  uint16_t len = chip0.tx_insert_idx[ chip0.curbuf ];
  uint8_t curbuf = chip0.curbuf;

  // switch to other buffer to accept more chars.
  chip0.curbuf++;
  if ( chip0.curbuf >= BLUEGIGA_BUFFER_NUM ) {
    chip0.curbuf = 0;
  }

  uint16_t ptr = bluegiga_sock_get16( TELEM_SOCKET, SOCK_TX_WR );
  uint16_t offset = ptr & SMASK;
  uint16_t dstAddr = offset + SBASE[ TELEM_SOCKET ];

  chip0.tx_insert_idx[ chip0.curbuf ] = 0;
  chip0.tx_extract_idx[ chip0.curbuf ] = 0;

  if (offset + len > SSIZE) {
    // Wrap around circular buffer
    uint16_t size = SSIZE - offset;
    bluegiga_set_buffer( dstAddr, &chip0.tx_buf[curbuf][0], size );
    bluegiga_set_buffer( SBASE[ TELEM_SOCKET ], &chip0.tx_buf[curbuf][0] + size, len - size);
  }
  else {
    bluegiga_set_buffer( dstAddr, &chip0.tx_buf[curbuf][0], len);
  }

  // Reset write pointer
  ptr += len;
  bluegiga_sock_set( TELEM_SOCKET, SOCK_TX_WR, ptr >> 8 );
  bluegiga_sock_set( TELEM_SOCKET, SOCK_TX_WR+1, ptr & 0xFF );

  // send
  bluegiga_sock_set( TELEM_SOCKET, SOCK_CR, SOCK_SEND );

  uint8_t complete = bluegiga_sock_get( TELEM_SOCKET, SOCK_CR);
  while ( complete != 0x00 ) {
    complete = bluegiga_sock_get( TELEM_SOCKET, SOCK_CR);
  }
}

uint16_t bluegiga_receive( uint8_t *buf, uint16_t len __attribute__((unused))) {
  uint8_t head[8];
  uint16_t data_len=0;
  uint16_t ptr=0;

  // Get socket read pointer
  ptr = bluegiga_sock_get16( CMD_SOCKET, SOCK_RXRD );
  bluegiga_read_data( CMD_SOCKET, (uint8_t *)ptr, head, 0x08);
  ptr += 8;
  data_len = head[6];
  data_len = (data_len << 8) + head[7];

  // read data from buffer.
  bluegiga_read_data( CMD_SOCKET, (uint8_t *)ptr, buf, data_len); // data copy.
  ptr += data_len;

  return data_len;
}

static void bluegiga_read_data( uint8_t s __attribute__((unused)), volatile uint8_t *src, volatile uint8_t *dst, uint16_t len ) {
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = (uint16_t)src & RMASK;
  src_ptr = RBASE[CMD_SOCKET] + src_mask;

  if( (src_mask + len) > RSIZE ) {
    size = RSIZE - src_mask;
    bluegiga_read(src_ptr, (uint8_t *)dst, size);
    dst += size;
    bluegiga_read(RBASE[CMD_SOCKET], (uint8_t *) dst, len - size);
  } else {
    bluegiga_read(src_ptr, (uint8_t *) dst, len);
  }
}

static uint16_t bluegiga_read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
  for (int i=0; i<_len; i++) {
    _buf[i] = bluegiga_get( _addr );
    _addr++;
  }
  return _len;
}

