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

#include "led.h"

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

struct bluegiga_periph bluegiga_p;
struct spi_transaction bluegiga_spi;

// Functions for the generic device API
static int true_function (struct bluegiga_periph* p __attribute__((unused)), uint8_t len __attribute__((unused)))
{
  return TRUE;
}
static void dev_transmit (struct bluegiga_periph* p __attribute__((unused)), uint8_t byte)
{
  bluegiga_transmit (byte);
}
static void dev_send (struct bluegiga_periph* p __attribute__((unused)))
{
  bluegiga_send ();
}

void bluegiga_init (void)
{

  LED_INIT(3);

  // configure the SPI bus.
  bluegiga_spi.input_buf      = bluegiga_p.work_rx;
  bluegiga_spi.output_buf     = bluegiga_p.work_tx;
  bluegiga_spi.input_length   = 20;
  bluegiga_spi.output_length  = 20;
  bluegiga_spi.slave_idx      = BLUEGIGA_SLAVE_IDX;
  bluegiga_spi.select         = SPISelectUnselect;
  bluegiga_spi.cpol           = SPICpolIdleHigh;
  bluegiga_spi.cpha           = SPICphaEdge1;
  bluegiga_spi.dss            = SPIDss8bit;
  bluegiga_spi.bitorder       = SPIMSBFirst;
  bluegiga_spi.cdiv           = SPIDiv64;

  // initialize peripheral variables
  bluegiga_p.tx_running       = 0;
  bluegiga_p.rx_insert_idx    = 0;
  bluegiga_p.rx_extract_idx   = 0;
  bluegiga_p.tx_insert_idx    = 0;
  bluegiga_p.tx_extract_idx   = 0;

  for (int i = 0; i < bluegiga_spi.input_length; i++)
  {
    bluegiga_p.work_rx[i] = 0;
    bluegiga_p.work_tx[i] = 0;
  }

  // Configure generic device
  bluegiga_p.device.periph    = (void *) (&bluegiga_p);
  bluegiga_p.device.check_free_space = (check_free_space_t) true_function;
  bluegiga_p.device.transmit  = (transmit_t) dev_transmit;
  bluegiga_p.device.send_message = (send_message_t) dev_send;

  // set DRDY interrupt pin for spi master triggered on falling edge
  gpio_setup_output (BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
  gpio_set(BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

  // register spi slave read for transaction
  spi_slave_register (&(BLUEGIGA_SPI_DEV), &bluegiga_spi);
}

/* safe increment of circular buffer */
void bluegiga_increment_buf(uint8_t *buf_idx, uint8_t len)
{
  *buf_idx = (*buf_idx + len) % BLUEGIGA_BUFFER_SIZE;
}

/* Add one byte to the end of tx circular buffer */
void bluegiga_transmit (uint8_t data)
{
  if (BlueGigaCheckFreeSpace() && bluegiga_p.tx_running)
    {
      bluegiga_p.tx_buf[bluegiga_p.tx_insert_idx] = data;
      bluegiga_increment_buf(&bluegiga_p.tx_insert_idx, 1);
    }
}

/* Send data in transmit buffer to spi master */
void bluegiga_send ()
{
  //if(bluegiga_spi.status != SPITransPending)
  //  return;

  uint8_t packet_len;

  // check data available in buffer to send
  packet_len = ((bluegiga_p.tx_insert_idx - bluegiga_p.tx_extract_idx + BLUEGIGA_BUFFER_SIZE) % BLUEGIGA_BUFFER_SIZE);
  if (packet_len > 19)
    packet_len = 19;

  if (packet_len)
  {
    uint8_t i;
    // attach header with data length of real data in 20 char data string
    bluegiga_p.work_tx[0] = packet_len;

    // copy data from working buffer to spi output buffer
    for (i = 0; i < packet_len; i++)
      bluegiga_p.work_tx[i + 1] = bluegiga_p.tx_buf[(bluegiga_p.tx_extract_idx + i) % BLUEGIGA_BUFFER_SIZE];
    bluegiga_increment_buf(&bluegiga_p.tx_extract_idx, packet_len);

    // clear unused bytes
    for (i = packet_len + 1; i < bluegiga_spi.output_length; i++)
      bluegiga_p.work_tx[i] = 0;

    // Test data integrity
    for (i = 0; i < bluegiga_spi.output_length; i++)
      bluegiga_p.work_tx[i] = i+1;

    // Now send off spi transaction!
    // trigger interrupt on BlueGiga to listen on spi
    gpio_clear (BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);
    while(bluegiga_spi.status != SPITransSuccess);
    // reset interrupt pin
    gpio_set (BLUEGIGA_DRDY_GPIO, BLUEGIGA_DRDY_GPIO_PIN);

    // clear tx buffer
    for (i = 0; i < bluegiga_spi.output_length; i++)
      bluegiga_p.work_tx[i] = 0;
  }
}

/* read data from dma if available */
void bluegiga_receive ( void )
{
  if (bluegiga_spi.status == SPITransSuccess)
  {
    LED_TOGGLE(3);
    bluegiga_spi.status = SPITransDone;
    if (!bluegiga_p.tx_running) bluegiga_p.tx_running = 1;

    uint8_t packet_len = bluegiga_p.work_rx[0];

    if (packet_len > bluegiga_spi.input_length)
    {
      // connection lost event
      bluegiga_p.tx_running = 0;
      // reinitialize peripheral variables
      bluegiga_p.rx_insert_idx    = 0;
      bluegiga_p.rx_extract_idx   = 0;
      bluegiga_p.tx_insert_idx    = 0;
      bluegiga_p.tx_extract_idx   = 0;

      LED_OFF(3);

      spi_slave_register (&(BLUEGIGA_SPI_DEV), &bluegiga_spi);

      return;
    }

    uint8_t i;
    for (i = 0; i < packet_len; i++)
    {
      bluegiga_p.rx_buf[(bluegiga_p.rx_insert_idx + i) % BLUEGIGA_BUFFER_SIZE] = bluegiga_p.work_rx[i + 1];
    }
    bluegiga_increment_buf(&bluegiga_p.rx_insert_idx, packet_len);

    // clear rx buffer
    for (i = 0; i < bluegiga_spi.output_length; i++)
      bluegiga_p.work_rx[i] = 0;

    // register spi slave read for next transaction
    spi_slave_register (&(BLUEGIGA_SPI_DEV), &bluegiga_spi);
  }
}
