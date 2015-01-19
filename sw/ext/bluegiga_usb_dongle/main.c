//
// Bluegiga's Bluetooth Smart Demo Application
// Contact: support@bluegiga.com.
//
// This is free software distributed under the terms of the MIT license reproduced below.
//
// Copyright (c) 2012, Bluegiga Technologies
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>

#include "cmd_def.h"
#include "uart.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int sock;
int addr_len, bytes_read;
char recv_data[1024], send_data[1024];
struct sockaddr_in server_addr, client_addr;

//#define DEBUG

#define CLARG_PORT 1
#define CLARG_ACTION 2

#define UART_TIMEOUT 100

#define MAX_DEVICES 8
int found_devices_count = 0;
bd_addr found_devices[MAX_DEVICES];
int connected[] = {0, 0, 0, 0, 0, 0, 0, 0};

signed char rssi[] = {0, 0, 0, 0, 0, 0, 0, 0};

int connect_all = 0;
uint8 MAC_ADDR[] = {0x00, 0x00, 0x1e, 0x80, 0x07, 0x00};//{0x00,0x00,0x2d,0x80,0x07,0x00};

enum actions {
  action_none,
  action_scan,
  action_connect,
  action_info,
  action_connect_all,
};
enum actions action = action_none;

typedef enum {
  state_disconnected,
  state_connecting,
  state_connected,
  state_finding_services,
  state_finding_attributes,
  state_listening_measurements,
  state_scanning,
  state_finish,
  state_last
} states;
states state = state_disconnected;

const char *state_names[state_last] = {
  "disconnected",
  "connecting",
  "connected",
  "finding_services",
  "finding_attributes",
  "listening_measurements",
  "scanning",
  "finish"
};

#define FIRST_HANDLE 0x0001
#define LAST_HANDLE  0xffff

#define DRONE_SERVICE_UUID            0x77cc
#define DRONE_DATA_UUID         0x3cc1
#define DRONE_DATA_CONFIG_UUID        0x2902
#define DRONE_BROADCAST_UUID        0x25ec

uint8 primary_service_uuid[] = {0x00, 0x28};

uint16 drone_handle_start = 0,
       drone_handle_end = 0,
       drone_handle_measurement = 0,
       drone_handle_configuration = 0,
       drone_handle_broadcast = 0;

bd_addr connect_addr;

FILE *fp;

void usage(char *exe)
{
  printf("%s <COMx|list> <scan|address>\n", exe);
}

void change_state(states new_state)
{
#ifdef DEBUG
  printf("DEBUG: State changed: %s --> %s\n", state_names[state], state_names[new_state]);
#endif
  state = new_state;
}

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return Zero if addresses are equal
 */
int cmp_bdaddr(bd_addr first, bd_addr second)
{
  int i;
  for (i = 0; i < sizeof(bd_addr); i++) {
    if (first.addr[i] != second.addr[i]) { return 1; }
  }
  return 0;
}

int cmp_addr(uint8 first[], uint8 second[])
{
  int i;
  for (i = 5; i >= 0; i--) {
    if (first[i] != second[i]) { return 5 - i; }
  }
  return 6;
}

void print_bdaddr(bd_addr bdaddr)
{
  printf("%02x:%02x:%02x:%02x:%02x:%02x",
         bdaddr.addr[5],
         bdaddr.addr[4],
         bdaddr.addr[3],
         bdaddr.addr[2],
         bdaddr.addr[1],
         bdaddr.addr[0]);
}

void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
  printf("Incoming packet: ");
  int i;
  for (i = 0; i < sizeof(*hdr); i++) {
    printf("%02x ", ((unsigned char *)hdr)[i]);
  }
  for (i = 0; i < hdr->lolen; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
}

void output(uint8 len1, uint8 *data1, uint16 len2, uint8 *data2)
{
  if (uart_tx(len1, data1) || uart_tx(len2, data2)) {
    printf("ERROR: Writing to serial port failed\n");
    exit(1);
  }
}
int count_packet = 0;
int read_message(int timeout_ms)
{
  unsigned char data[256]; // enough for BLE
  struct ble_header hdr;
  int r;

  r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, UART_TIMEOUT);
  if (!r) {
    return -1; // timeout
  } else if (r < 0) {
    printf("ERROR: Reading header failed. Error code:%d\n", r);
    return 1;
  }

  if (hdr.lolen) {
    r = uart_rx(hdr.lolen, data, UART_TIMEOUT);
    if (r <= 0) {
      printf("ERROR: Reading data failed. Error code:%d\n", r);
      return 1;
    }
  }

  const struct ble_msg *msg = ble_get_msg_hdr(hdr);

#ifdef DEBUG
  print_raw_packet(&hdr, data);
#endif

  if (!msg) {
    printf("ERROR: Unknown message received\n");
    exit(1);
  }

  msg->handler(data);

  count_packet++;
  return 0;
}

void enable_indications(uint8 connection_handle, uint16 client_configuration_handle)
{
  ble_cmd_sm_encrypt_start(connection_handle, 1);   // encrpyt connection (not required)

  uint8 configuration[] = {0x01, 0x00}; // enable indications
  ble_cmd_attclient_attribute_write(connection_handle, drone_handle_configuration, 2, &configuration);
}

void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
  printf("Build: %u, protocol_version: %u, hardware: ", msg->build, msg->protocol_version);
  switch (msg->hw) {
    case 0x01: printf("BLE112"); break;
    case 0x02: printf("BLED112"); break;
    default: printf("Unknown: %d", msg->hw);
  }
  printf("\n");

  if (action == action_info) { change_state(state_finish); }
}

void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{

  int i = 0;
  char *name = NULL;

  if (cmp_addr(msg->sender.addr, MAC_ADDR) >= 4) {
	  //if (found_devices_count >= MAX_DEVICES) { printf("MAX!"); }

	  // Check if this device already found
	  while (i < found_devices_count) {
		if (!cmp_bdaddr(msg->sender, found_devices[i])) {
		  found_devices_count--;
		  break;
		}
		i++;
	  }

	  memcpy(found_devices[i].addr, msg->sender.addr, sizeof(bd_addr));
	  found_devices_count++;

	  rssi[i] = msg->rssi;
	  printf("i = %d rssi: %d found devices %d\n", i, rssi[i], found_devices_count);
	  bytes_read = recvfrom(sock, recv_data, 1024, MSG_DONTWAIT, (struct sockaddr *)&client_addr, &addr_len);
	  sendto(sock, rssi, found_devices_count, MSG_DONTWAIT, (struct sockaddr *)&client_addr, sizeof(client_addr));
  }
  return;
  printf("New device found: ");

  // Parse data
  /*    for (i = 0; i < msg->data.len; ) {
          int8 len = msg->data.data[i++];
          if (!len) continue;
          if (i + len > msg->data.len) break; // not enough data
          uint8 type = msg->data.data[i++];
          switch (type) {
          case 0x09:
              name = malloc(len);
              memcpy(name, msg->data.data + i, len - 1);
              name[len - 1] = '\0';
          }

          i += len - 1;
      }*/

  //print_bdaddr(msg->sender);
  //printf(" RSSI:%d", msg->rssi);

  //printf(" Name:");
  //if (name) printf("%s", name);
  //else printf("Unknown");
  //printf("\n");

  if (cmp_addr(msg->sender.addr, MAC_ADDR) >= 4) {

    printf("Trying to connect to "); print_bdaddr(msg->sender); printf("\n");
    change_state(state_connecting);
    // connection interval must be divisible by number of connection * 2.5ms
    double min = found_devices_count * 2;
    if (min < 6) {
      min = min * (6 - min);
    }
    ble_cmd_gap_connect_direct(&msg->sender.addr, gap_address_type_public, (int)min, (int)min * 2, 100, 9);
    for (i = 0; i < found_devices_count - 1; i++) {
      ble_cmd_connection_update(i, (int)min, (int)min * 2, 9, 100);
    }
  }

  free(name);
}

void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
  // Encrypted previous connection
  if (msg->flags & connection_encrypted) {
    printf("Connection with %d is encrypted\n", msg->connection);
  }

  if (msg->flags & connection_completed) {
    change_state(state_connected);
    printf("Connected %d\n", msg->connection);
    connected[msg->connection] = 1;

    fprintf(fp, "# %x = %d\n", msg->address.addr[0], msg->connection);

    // Handle for Drone Data configuration already known
    if (drone_handle_configuration) {
      change_state(state_listening_measurements);
      enable_indications(msg->connection, drone_handle_configuration);
    }
    // Find primary services
    else {
      change_state(state_finding_services);
      ble_cmd_attclient_read_by_group_type(msg->connection, FIRST_HANDLE, LAST_HANDLE, 2, primary_service_uuid);
    }
  }

}

void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
  if (msg->uuid.len == 0) { return; }
  uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

  if (state == state_finding_services) {
    printf("length: %d uuid: %x\n", msg->uuid.len, uuid);
  }

  // First data service found
  if (state == state_finding_services && uuid == DRONE_SERVICE_UUID && drone_handle_start == 0) {
    drone_handle_start = msg->start;
    drone_handle_end = msg->end;
  }
}

void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
  if (state == state_finding_services) {
    // Data service not found
    if (drone_handle_start == 0) {
      printf("No Drone service found\n");
      change_state(state_finish);
    }
    // Find drone service attributes
    else {
      change_state(state_finding_attributes);
      ble_cmd_attclient_find_information(msg->connection, drone_handle_start, drone_handle_end);
    }
  } else if (state == state_finding_attributes) {
    // Client characteristic configuration not found
    if (drone_handle_configuration == 0) {
      printf("No Client Characteristic Configuration found for Drone Data service\n");
      change_state(state_finish);
    }
    // Enable drone notifications
    else {
      change_state(state_listening_measurements);
      enable_indications(msg->connection, drone_handle_configuration);
    }
  }
}

void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
//TODO: add different rule for 16
  if (msg->uuid.len == 2 || msg->uuid.len == 16) {
    uint32 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];
    if (uuid == DRONE_DATA_CONFIG_UUID) {
      drone_handle_configuration = msg->chrhandle;
    } else if (uuid == DRONE_DATA_UUID) {
      drone_handle_measurement = msg->chrhandle;
    } else if (uuid == DRONE_BROADCAST_UUID) {
      drone_handle_broadcast = msg->chrhandle;
    }
  }
}


void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{

}

void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{
  connected[msg->connection] = 0;
  // remove found device from list
  change_state(state_disconnected);
  printf("Connection terminated\n");
  if (connect_all) {
    change_state(state_scanning);
    ble_cmd_gap_discover(gap_discover_observation);
    return;
  }

  change_state(state_connecting);
  ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 40, 60, 100, 0);
  //change_state(state_finish);
}

int kbhit(void)
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET(STDIN_FILENO, &rdfs);

  select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}

int main(int argc, char *argv[])
{
  char *uart_port = "";

  //ble_cmd_sm_set_bondable_mode(1);

  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("Socket");
    exit(1);
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(5000);
  server_addr.sin_addr.s_addr = INADDR_ANY;
  bzero(&(server_addr.sin_zero), 8);


  if (bind(sock, (struct sockaddr *)&server_addr,
           sizeof(struct sockaddr)) == -1) {
    perror("Bind");
    exit(1);
  }

  addr_len = sizeof(struct sockaddr);

  // Not enough command-line arguments
//    if (argc <= CLARG_PORT) {
//        usage(argv[0]);
//        return 1;
//    }

  // COM port argument
  /*    if (argc > CLARG_PORT) {
          if (strcmp(argv[CLARG_PORT], "list") == 0) {
              uart_list_devices();
              return 1;
          }
          else {
              uart_port = argv[CLARG_PORT];
          }
      }*/

  uart_port = "/dev/ttyACM0";

  /*// Action argument
  if (argc > CLARG_ACTION) {
      int i;
      for (i = 0; i < strlen(argv[CLARG_ACTION]); i++) {
          argv[CLARG_ACTION][i] = tolower(argv[CLARG_ACTION][i]);
      }

      if (strcmp(argv[CLARG_ACTION], "scan") == 0) {
          action = action_scan;
    connect_all = 1;
      }
      else if (strcmp(argv[CLARG_ACTION], "info") == 0) {
          action = action_info;
      }
  else if (strcmp(argv[CLARG_ACTION], "all") == 0) {
    connect_all = 1;
          action = action_scan;
      }
      else {
          int i;
          short unsigned int addr[6];
          if (sscanf(argv[CLARG_ACTION],
                  "%02hx:%02hx:%02hx:%02hx:%02hx:%02hx",
                  &addr[5],
                  &addr[4],
                  &addr[3],
                  &addr[2],
                  &addr[1],
                  &addr[0]) == 6) {

              for (i = 0; i < 6; i++) {
                  connect_addr.addr[i] = addr[i];
              }
              action = action_connect;
          }
      }
  }
  if (action == action_none) {
      usage(argv[0]);
      return 1;
  }
  */
  bglib_output = output;

  if (uart_open(uart_port)) {
    printf("ERROR: Unable to open serial port - %s\n", strerror(errno));
    return 1;
  }

  // Reset dongle to get it into known state
  ble_cmd_system_reset(0);
  uart_close();
  do {
    usleep(500000); // 0.5s
  } while (uart_open(uart_port));

  /*// Execute action
  if (action == action_scan) {
      ble_cmd_gap_discover(gap_discover_generic);
  }
  else if (action == action_info) {
      ble_cmd_system_get_info();
  }
  else if (action == action_connect) {
      printf("Trying to connect\n");
      change_state(state_connecting);
      ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 16, 32, 100, 9);
  }*/

  ble_cmd_gap_discover(gap_discover_observation);

  int counter = 0;
  // Message loop
  while (1) {
    if (read_message(UART_TIMEOUT) > 0) { break; }
    if (counter > 7) {
      ble_cmd_gap_end_procedure();
      ble_cmd_gap_set_mode(0x80, gap_scannable_non_connectable);
    }
    if (counter++ > 8) {
      ble_cmd_gap_discover(gap_discover_observation);
      counter = 0;
    }
    if (kbhit()) {
      getchar();
      break;
    }
  }

  uart_close();

  return 0;
}
