
#pragma once

#include <esp_err.h>

#ifndef __APP_DRIVER_H__
#define __APP_DRIVER_H__

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

// DHT22 Values
#define DHT22_GPIO_PIN GPIO_NUM_5
#define DEFAULT_TEMPERATURE_VALUE 1000
#define DEFAULT_HUMIDITY_VALUE 1000

#define DEFAULT_MEASURE_INTERVAL 120000

//ThingSeak Values
#define THINGSPEAK_WRITE_API_KEY  "406WGRX36U0HQTL1"
#define THINGSPEAK_URL "https://api.thingspeak.com/update?"

//WIFI
//#define ESP_WIFI_SSID "MEO-9E24A0"
//#define ESP_WIFI_PASS "99419955a3"
#define ESP_WIFI_SSID "OpenFCT"
#define ESP_WIFI_PASS ""
//#define ESP_WIFI_SSID "Fil2k"
//#define ESP_WIFI_PASS "filipe2003"

extern esp_rmaker_device_t *temp_sensor_device;
extern esp_rmaker_device_t *humidity_sensor_device;
extern esp_rmaker_device_t *lux_sensor_device;
extern esp_rmaker_device_t *pir_sensor_device;
typedef void *app_driver_handle_t;

#endif // __APP_DRIVER_H__
