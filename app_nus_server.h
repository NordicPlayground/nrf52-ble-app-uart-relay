#ifndef __APP_NUS_SERVER_H
#define __APP_NUS_SERVER_H

#include <stdint.h>
#include "nrf.h"
#include "ble.h"

typedef void (*app_nus_server_on_data_received_t)(const uint8_t *data_ptr, uint16_t data_length);

void app_nus_server_send_data(uint8_t *data_array, uint16_t length);

void app_nus_server_ble_evt_handler(ble_evt_t const * p_ble_evt);

void app_nus_server_init(app_nus_server_on_data_received_t on_data_received);

#endif
