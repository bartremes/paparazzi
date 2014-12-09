/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/com/bluegiga.c"
 * @author C. De Wagter
 * Communicate through BlueGiga SPI modules
 */

#include "modules/com/bluegiga.h"

#include "mcu_periph/gpio.h"
#include "mcu_periph/spi.h"


/* The structure for the cyrf6936 chip that handles all the buffers and requests */
struct BlueGigaDev {
  struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
  struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
  uint8_t input_buf[32];                    /**< The input buffer for the SPI transaction */
  uint8_t output_buf[32];                   /**< The output buffer for the SPI transaction */
};

struct BlueGigaDev bluegiga_dev;

void bluegiga_init()
{
  gpio_setup_output(GPIOC, GPIO6);

  bluegiga_dev.spi_p = &spi2;

  // Set the spi transaction
  bluegiga_dev.spi_t.cpol = SPICpolIdleHigh;
  bluegiga_dev.spi_t.cpha = SPICphaEdge1;
  bluegiga_dev.spi_t.dss = SPIDss8bit;
  bluegiga_dev.spi_t.bitorder = SPIMSBFirst;
  bluegiga_dev.spi_t.cdiv = SPIDiv64;

  bluegiga_dev.spi_t.input_length = 20;
  bluegiga_dev.spi_t.output_length = 20;
  bluegiga_dev.spi_t.input_buf = bluegiga_dev.input_buf;
  bluegiga_dev.spi_t.output_buf = bluegiga_dev.output_buf;
  bluegiga_dev.spi_t.select = SPISelectUnselect;

  for (int i=0;i<32;i++)
  {
    bluegiga_dev.input_buf[i] = i;
    bluegiga_dev.output_buf[i] = i;
  }

  spi_slave_register(bluegiga_dev.spi_p, &(bluegiga_dev.spi_t));
}

uint8_t counter = 0;
void bluegiga_periodic()
{
  //uint16_t rx_value = 0x42;

  gpio_toggle(GPIOC, GPIO6);
  //spi_set_nss_low(SPI2);
  //spi_send(SPI2, (uint8_t) counter);
  //rx_value = spi_read(SPI2);


  //if(bluegiga_dev.spi_t.status != SPITransDone)
  //  return;

  //bluegiga_dev.spi_t.output_length = 1;
  //bluegiga_dev.spi_t.input_length = 1;
  //bluegiga_dev.output_buf[0]++;

  // Submit the transaction
  //spi_submit(bluegiga_dev.spi_p, &(bluegiga_dev.spi_t));
}

void bluegiga_event()
{
  //if ((SPI_SR(SPI2) & SPI_SR_TXE))
  //  spi_send(SPI2, counter++);
  if(bluegiga_dev.spi_t.status == SPITransSuccess)
  {
    gpio_toggle(GPIOC, GPIO6);
    bluegiga_dev.output_buf[0] = counter++;
    bluegiga_dev.spi_t.input_length = 20;
    bluegiga_dev.spi_t.output_length = 20;
    for (int i=1;i<32;i++)
    {
      bluegiga_dev.input_buf[i] = i;
      bluegiga_dev.output_buf[i] = i;
    }

    spi_slave_register(bluegiga_dev.spi_p, &(bluegiga_dev.spi_t));
  }
}


