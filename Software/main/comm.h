#ifndef _COMM_H__
#define _COMM_H__

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
#define MAX_STA_CONN    2
#define UDP_PORT        2390
#define TELEMETRY_PORT  2391

#define CRTP_PORT_COMMANDER 3
#define CRTP_PORT_TUNING    0x0F
#define CRTP_DATA_SIZE      31

typedef struct __attribute__((packed)) {
    uint8_t channel : 4;
    uint8_t port : 4;
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
    uint8_t reserved;
} crtp_packet_t;

extern crtp_packet_t pkt;
extern uint8_t rx_buffer[1024];
extern int len;

void wifi_init(void);
void UDP_server_init(struct sockaddr_in *server_addr, struct sockaddr_in *client_addr, 
    socklen_t *socklen, int *sock);

#endif