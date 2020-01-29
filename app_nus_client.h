#ifndef __APP_NUS_CLIENT_H
#define __APP_NUS_CLIENT_H

#include <stdint.h>
#include "ble.h"
#include "nrf.h"

typedef void (*app_nus_client_on_data_received_t)(const uint8_t *data_ptr, uint16_t data_length);

uint32_t app_nus_client_send_data(const uint8_t *data_array, uint16_t length);

void app_nus_client_ble_evt_handler(ble_evt_t const * p_ble_evt);

void app_nus_client_init(app_nus_client_on_data_received_t on_data_received);

#endif
