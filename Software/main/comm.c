#include "comm.h"

crtp_packet_t pkt;
uint8_t rx_buffer[1024];
int len;

static const char *TAG = "wifi_softAP";
/**
 * @brief Event handler for Wi-Fi AP events
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
    }
}

void wifi_init(void)
{
    // Initialize NVS 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the TCP/IP stack and the event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default WiFi AP interface
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    // Stop DHCP server temporarily
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    // Set static IP
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = ipaddr_addr("192.168.43.42");   // DEVICE_ADDRESS
    ip_info.gw.addr = ipaddr_addr("192.168.43.1");    // Gateway
    ip_info.netmask.addr = ipaddr_addr("255.255.255.0");
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));

    // Restart DHCP server
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register WiFi event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    // Configure WiFi AP settings
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASSWORD,
            .channel = WIFI_CHANEL,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false
            },
        },
    };
    
    // Set WiFi mode and configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "Wi-Fi AP started");
    ESP_LOGI(TAG, "SSID:%s password:%s channel:%d, IP:%s",
             WIFI_SSID, WIFI_PASSWORD, WIFI_CHANEL, "192.168.43.42");
}

void UDP_server_init(struct sockaddr_in *server_addr, struct sockaddr_in *client_addr, 
    socklen_t *socklen, int *sock)
{

    if (*sock < 0) {
        ESP_LOGE("ESP32_UDP_AP", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    (*server_addr).sin_addr.s_addr = htonl(INADDR_ANY);
    (*server_addr).sin_family = AF_INET;
    (*server_addr).sin_port = htons(UDP_PORT);

    if (bind(*sock, (struct sockaddr *)server_addr, sizeof((*server_addr))) < 0) {
        ESP_LOGE("ESP32_UDP_AP", "Socket bind failed: errno %d", errno);
        close(*sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("ESP32_UDP_AP", "UDP server listening on port %d", UDP_PORT);
}