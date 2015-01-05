/*
 * Copyright (C) 2014 Kirk Scheper <kirkscheper@gmail.com>
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
 * @file subsystems/datalink/bluegiga.c
 * datalink implementation for the BlueGiga Bleutooth radio chip trough SPI
 */

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/bluegiga.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"

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
uint8_t bluegiga_rx_buf[BLUEGIGA_RX_BUFFER_SIZE];

struct spi_transaction bluegiga_spi;

// Functions for the generic device API
static int true_function(struct bluegiga_periph* p __attribute__((unused)), uint8_t len __attribute__((unused))) { return TRUE; }
static void dev_transmit(struct bluegiga_periph* p __attribute__((unused)), uint8_t byte) {  bluegiga_transmit(byte); }
static void dev_send(struct bluegiga_periph* p __attribute__((unused))) { bluegiga_send(); }

void bluegiga_init( void ) {

  // configure the SPI bus.
  bluegiga_spi.slave_idx = BLUEGIGA_SLAVE_IDX;
  bluegiga_spi.output_length = BLUEGIGA_TX_BUFFER_SIZE;
  bluegiga_spi.input_length = BLUEGIGA_RX_BUFFER_SIZE;
  bluegiga_spi.select = SPISelectUnselect;

  bluegiga_spi.cpol = SPICpolIdleHigh;
  bluegiga_spi.cpha = SPICphaEdge1;
  bluegiga_spi.dss = SPIDss8bit;
  bluegiga_spi.bitorder = SPIMSBFirst;
  bluegiga_spi.cdiv = SPIDiv64;

  bluegiga_spi.input_buf = &chip0.work_rx[0];
  bluegiga_spi.output_buf = &chip0.work_tx[0];

  chip0.curbuf = 0;

  // set DRDY pin
  gpio_setup_output(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

  // Configure generic device
  chip0.device.periph = (void *)(&chip0);
  chip0.device.check_free_space = (check_free_space_t) true_function;
  chip0.device.transmit = (transmit_t) dev_transmit;
  chip0.device.send_message = (send_message_t) dev_send;

  // register spi slave read for transaction
  spi_slave_register( &(BLUEGIGA_SPI_DEV), &bluegiga_spi );
}

bool_t bluegiga_check_free_space(int len)
{
  return chip0.tx_insert_idx[ chip0.curbuf ] + len > BLUEGIGA_TX_BUFFER_SIZE;
}

void bluegiga_transmit( uint8_t data ) {
  uint16_t temp = (chip0.tx_insert_idx[ chip0.curbuf ] + 1) % BLUEGIGA_TX_BUFFER_SIZE;

  if (!bluegiga_check_free_space(1)) {
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

  chip0.tx_insert_idx[ chip0.curbuf ] = 0;

  // reset output buffer
  int i;
  for (i=0; i<bluegiga_spi.output_length; i++)
    bluegiga_spi.output_buf[i]=0;

  // copy data from working buffer to spi output buffer
  memcpy(bluegiga_spi.output_buf, &chip0.tx_buf[curbuf][0], len);

  // trigger interrupt on BlueGiga to listen on spi
  gpio_clear(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

  // send data over spi slave
  spi_submit( &(BLUEGIGA_SPI_DEV), &bluegiga_spi );

  // reset interrupt pin
  gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
}

bool_t bluegiga_ch_available() {return bluegiga_spi.status == SPITransSuccess;}

uint16_t bluegiga_receive( uint8_t *buf, uint16_t len __attribute__((unused))) {

  uint16_t data_len=20;

  memcpy(buf, chip0.work_rx, data_len);

  // register spi slave read for next transaction
  spi_slave_register( &(BLUEGIGA_SPI_DEV), &bluegiga_spi );

  return data_len;
}
