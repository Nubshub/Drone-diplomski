#ifndef _WIFI_CONFIG_H__
#define _WIFI_CONFIG_H__

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define WIFI_SSID       "ESP32_Drone"
#define WIFI_PASSWORD   "12345678"
#define WIFI_CHANEL     1
#define MAX_STA_CONN    1
#define UDP_PORT        2390

#define CRTP_PORT_COMMANDER 3
#define CRTP_DATA_SIZE 31

void wifi_init(void);

#endif