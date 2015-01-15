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
#include "led.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

signed char rssi[8];
char k_rssi = 0;

/* The structure for the cyrf6936 chip that handles all the buffers and requests */
/*struct BlueGigaDev {
  int activated;
  struct spi_periph *spi_p;                 *< The SPI peripheral for the connection
  struct spi_transaction spi_t;             *< The SPI transaction used for the writing and reading of registers
  uint8_t input_buf[32];                    *< The input buffer for the SPI transaction
  uint8_t output_buf[32];                   *< The output buffer for the SPI transaction
};

struct BlueGigaDev bluegiga_dev;
*/
int sock,bytes_recv,sin_size;
struct sockaddr_in server_addr;
struct hostent *host;
char send_data[1024],recv_data[1024];

void bluegiga_com_init()
{
  /*gpio_setup_output(GPIOC, GPIO6);

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

  bluegiga_dev.activated = 0;
  LED_INIT(3);

  spi_slave_register(bluegiga_dev.spi_p, &(bluegiga_dev.spi_t));*/

  host= (struct hostent *) gethostbyname((char *)"127.0.0.1");


  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
  {
  perror("socket");
  exit(1);
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(5000);
  server_addr.sin_addr = *((struct in_addr *)host->h_addr);
  bzero(&(server_addr.sin_zero),8);
  sin_size = sizeof(struct sockaddr);

  strcpy(send_data,"1");

}

uint8_t counter = 0;
void bluegiga_com_periodic()
{
  sendto(sock, send_data, strlen(send_data), MSG_DONTWAIT,
                (struct sockaddr *)&server_addr, sizeof(struct sockaddr));

  bytes_recv = recvfrom(sock,recv_data,1024,MSG_DONTWAIT,(struct sockaddr *)&server_addr,(socklen_t *)&sin_size);
  // recv_data[bytes_recv]= '\0';

  if (bytes_recv > 0){
    k_rssi = bytes_recv;
    printf("Paparazzi rssi: ");
    for (int i = 0; i< k_rssi; i++){
      rssi[i] = (signed char) recv_data[i];
      printf("%d ",rssi[i]);
    }
    printf("\n");
  }

  //uint16_t rx_value = 0x42;

  //if (bluegiga_dev.activated)
  // gpio_toggle(GPIOC, GPIO6);
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

void bluegiga_com_event()
{
  //if ((SPI_SR(SPI2) & SPI_SR_TXE))
  //  spi_send(SPI2, counter++);
/*  if(bluegiga_dev.spi_t.status == SPITransSuccess)
  {
      //if ( counter % 5 )
	LED_TOGGLE(3);
    //if (!bluegiga_dev.activated)
    //  bluegiga_dev.activated = 1;

    //gpio_toggle(GPIOC, GPIO6);
    bluegiga_dev.output_buf[0] = counter++;
    bluegiga_dev.spi_t.input_length = 20;
    bluegiga_dev.spi_t.output_length = 20;
    for (int i=1;i<32;i++)
    {
      bluegiga_dev.input_buf[i] = i;
      bluegiga_dev.output_buf[i] = i;
    }

    spi_slave_register(bluegiga_dev.spi_p, &(bluegiga_dev.spi_t));
  }*/
}


