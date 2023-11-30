//SET TO 1 for master device and 0 for all other
#define IS_MASTER 0

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include "spi_flash_mmap.h"
#include <esp_http_server.h>
#include "esp_now_header.h"
#include "esp_now.h"
#include "esp_eth.h"
#include "esp_crc.h"
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_random.h"
#include "esp_mac.h"

typedef struct
{
    char str_value[4];
    long long_value;
} URL_t;

const char* web_homepage_string = "<!DOCTYPE html><html><head> <title>Light Dimmer Website</title> <style> /* Body */ body { text-align: center; padding: 15px; background-image: url('https://images.unsplash.com/photo-1505506874110-6a7a69069a08?ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D&auto=format&fit=crop&w=1887&q=80'); color: #FFD700; overflow: hidden;} /* Div */ div { margin-bottom: 200px; border-radius: 5px; } /* On/Off Button */ .onoffbutton { background-color: #6A5ACD; height: 100px; width: 100px; border-radius: 10px; color: #FFD700; margin-top: 10px; margin-bottom: 10px; } .onoffbutton:active { background-color: #AAAAAA; } /* Slider */ .slider { -webkit-appearance: none; --sliderOpac: 0.5; width: 100%; height: 8px; background: #d3d3d3; border-radius: 5px; } .slider::-webkit-slider-thumb { -webkit-appearance: none; width: 40px; height: 40px; background: url('https://illustoon.com/photo/dl/5764.png') no-repeat center; background-size: contain; border: none; cursor: pointer; opacity: var(--sliderOpac); } </style></head><body> <h1>Light Dimmer</h1> <div id=\"currentLight\"></div> <label>Select Light Level: </label> <span id=\"sliderVal\">50</span>% <div id=\"lightRange\"> <input type=\"range\" min=\"0\" max=\"100\" step=\"1\" value=\"50\" class=\"slider\" id=\"slider\" /> </div> <div> <input type=\"button\" value=\"Turn On\" class=\"onoffbutton\" id=\"onoff\"> </div> <script> const slider = document.getElementById('slider'); const sliderVal = document.getElementById('sliderVal'); slider.addEventListener(\"input\", event => { const sliderValue = slider.value; sliderVal.textContent = sliderValue; slider.style.setProperty('--sliderOpac', sliderValue / 100 + 0.2); }); slider.addEventListener(\"mouseup\", event => { const sliderValue = slider.value; let roundedValue; var httpRequest = new XMLHttpRequest(); if (sliderValue % 10 === 1 || sliderValue % 10 === 2 || sliderValue % 10 === 8 || sliderValue % 10 === 9) { roundedValue = Math.round(sliderValue / 10) * 10; } else { roundedValue = sliderValue; } sliderVal.textContent = roundedValue; httpRequest.open(\"GET\", \"/light_value?value=\" + roundedValue, true); httpRequest.send()}); const button = document.getElementById('onoff'); let button_val = 0; button.addEventListener(\"mouseup\", event => { var httpRequest = new XMLHttpRequest(); if (button_val === 0) { button_val = 1; button.value = 'Turn Off'; } else { button_val = 0; button.value = 'Turn On' } httpRequest.open(\"GET\", \"/on_off_value?value=\" + button_val, true); httpRequest.send() }); </script></body></html>";

/* Host WiFi Details.   for ee162 wifi        (hotspot)*/
#define ESP_HOST_WIFI_SSID      "team12"        //"EE162"         //"Sams phone"
#define ESP_HOST_WIFI_PASS      "Senior Design Sux"        //"Eceee162"      //"thisisgoingtobethepassword"
#define ESP_HOST_MAXIMUM_RETRY  15
#define CONFIG_ESPNOW_CHANNEL   11           //136             //6 //FOR MY HOTSPOT

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG_subsystem4 = "ESP Subsystem4";

static int s_retry_num = 0;

//Send Params:
#define CONFIG_ESPNOW_SEND_COUNT 1
#define CONFIG_ESPNOW_SEND_DELAY 200
#define CONFIG_ESPNOW_SEND_LEN sizeof(espnow_data_t)
#define ESPNOW_MAXDELAY 512

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t master_mac[ESP_NOW_ETH_ALEN] = { 0x84, 0x0D, 0x8E, 0xE3, 0xB8, 0x30};
static uint8_t other_mac_1[ESP_NOW_ETH_ALEN] = { 0x4C, 0x11, 0xAE, 0x70, 0xFD, 0x48};

static QueueHandle_t espnow_queue;

int req_light_level = 50;   //REQ LIGHT LVL GLOBAL VAR
bool device_power = true;   //ON/OFF GLOBAL VAR

// Function Declarations (Just so there are no errors)
static void espnow_deinit(espnow_send_param_t *send_param);
int espnow_data_parse(uint8_t *data, uint16_t data_len, int32_t *light_level, uint16_t *on_off, int *magic);
void espnow_data_prepare(espnow_send_param_t *send_param);    //NEEDS WORK
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static esp_err_t espnow_init(void);
void send_task(void *pvParameters);
static void receive_task(void *pvParameter);
esp_err_t get_req_handler(httpd_req_t *req);
esp_err_t send_web_page(httpd_req_t *req);
esp_err_t ReceiveSpeedFrom_Webclient_handler(httpd_req_t *req);
esp_err_t HomePage_req_handler(httpd_req_t *req);
httpd_handle_t Setup_HTTP_server(void);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

/* FreeRTOS event-handler to handle Wi-Fi and TCP/IP events.*/
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // if (s_retry_num < ESP_HOST_MAXIMUM_RETRY) 
        // {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_subsystem4, "retry to connect to the AP");
        // }
        // else
        // {
        //     xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        // }
        ESP_LOGI(TAG_subsystem4,"connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_subsystem4, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else
    {
        /* No Action.*/
    }
}

/* Function to configure ESP32 Wi-Fi.*/
void WiFi_InitIn_StationMode(void)
{
    s_wifi_event_group = xEventGroupCreate();

    /* lwIP is a widely used open-source TCP/IP stack designed for embedded systems.
    *ESP-NETIF is another library that encapsulates lwIP and provide another set of APIs.
    *The below function performs lwIP initialisation and create lwIP task. */
    ESP_ERROR_CHECK(esp_netif_init());

    /* There is something called as default event loop whose understanding isn't much.
    *This is required for handling WiFi events.*/
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Creating a network interface instance binding WiFi driver and
    *TCP/IP stack (esp-netif or lwIP). This is done by calling the below function.*/
    esp_netif_create_default_wifi_sta();

    /* Initializing the WiFi-Driver with the default configuration.*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Registering for callbacks that listens for WiFi and TCP/IP events.*/
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    /* Configuring the WiFi driver.*/
    wifi_config_t wifi_config = 
    {
        .sta = 
        {
            .ssid = ESP_HOST_WIFI_SSID,
            .password = ESP_HOST_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = "",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

    /* Start the WiFi and connect to the Access Point.*/
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG_subsystem4, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(TAG_subsystem4, "connected to ap SSID:%s password:%s",
                 ESP_HOST_WIFI_SSID, ESP_HOST_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG_subsystem4, "Failed to connect to SSID:%s, password:%s",
                 ESP_HOST_WIFI_SSID, ESP_HOST_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG_subsystem4, "UNEXPECTED EVENT");
    }
}

esp_err_t send_web_page(httpd_req_t *req)
{
    int response;
    response = httpd_resp_send(req, web_homepage_string, HTTPD_RESP_USE_STRLEN);
    return response;
}

esp_err_t HomePage_req_handler(httpd_req_t *req)
{
    return send_web_page(req);
}

esp_err_t ReceiveLightFrom_Webclient_handler(httpd_req_t *req) {
    ESP_LOGI(TAG_subsystem4, "RECIEVED GET REQ");
    int parsedValue;
    esp_err_t response_status = ESP_OK;

    if (sscanf(req->uri, "/light_value?value=%d", &parsedValue) == 1)
    {
        ESP_LOGI(TAG_subsystem4, "Extracted value: %d", parsedValue);

        // Create an instance of espnow_data_t to hold the parsed value
        espnow_data_t espnow_data;

        // Populate the data structure with the parsed value
        espnow_data.light_level = parsedValue;
        espnow_data.on_off = true; // Set other data fields if needed

        // Configure the espnow_send_param_t for unicast
        espnow_send_param_t send_param;
        send_param.unicast = true;
        send_param.broadcast = false;
        send_param.magic = esp_random(); // Set the magic number
        send_param.count = 1; // Send only one packet
        send_param.delay = 200; // Delay between sending two ESPNOW data packets in milliseconds
        send_param.len = sizeof(espnow_data);
        send_param.buffer = (uint8_t *)&espnow_data; // Point to the data structure
        memcpy(send_param.dest_mac, other_mac_1, ESP_NOW_ETH_ALEN); // Set the destination MAC address

        // Send the data using esp_now_send
        esp_err_t result = esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);

        if (result == ESP_OK) {
            ESP_LOGI(TAG_subsystem4, "Data sent successfully");
        } else {
            ESP_LOGE(TAG_subsystem4, "Failed to send data");
            response_status = ESP_FAIL;     //500 (internal server)
        }
    }
    else
    {
        ESP_LOGE(TAG_subsystem4, "Light_Value Not Found");
        response_status = ESP_ERR_NOT_FOUND;    //404 not found     (MIGHT NOT WORK)
    }

    req_light_level = parsedValue;

    // Send the HTTP response with the appropriate status code
    httpd_resp_send(req, NULL, 0);

    return response_status;
}

esp_err_t ReceiveOnOff_Webclient_handler(httpd_req_t *req) {
    ESP_LOGI(TAG_subsystem4, "RECIEVED GET REQ");
    int parsedValue;
    esp_err_t response_status = ESP_OK;

    if (sscanf(req->uri, "/on_off_value?value=%d", &parsedValue) == 1)
    {
        ESP_LOGI(TAG_subsystem4, "The value of on/off switch is: %s", parsedValue ? "On" : "Off");

        // Create an instance of espnow_data_t to hold the parsed value
        espnow_data_t espnow_data;

        // Populate the data structure with the parsed value
        espnow_data.on_off = parsedValue; // Set other data fields if needed

        // Configure the espnow_send_param_t for unicast
        espnow_send_param_t send_param;
        send_param.unicast = true;
        send_param.broadcast = false;
        send_param.magic = esp_random(); // Set the magic number
        send_param.count = 1; // Send only one packet
        send_param.delay = 200; // Delay between sending two ESPNOW data packets in milliseconds
        send_param.len = sizeof(espnow_data);
        send_param.buffer = (uint8_t *)&espnow_data; // Point to the data structure
        memcpy(send_param.dest_mac, other_mac_1, ESP_NOW_ETH_ALEN); // Set the destination MAC address

        // Send the data using esp_now_send
        esp_err_t result = esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);

        if (result == ESP_OK) {
            ESP_LOGI(TAG_subsystem4, "Data sent successfully");
        } else {
            ESP_LOGE(TAG_subsystem4, "Failed to send data");
            response_status = ESP_FAIL;     //500 (internal server)
        }
    }
    else
    {
        ESP_LOGE(TAG_subsystem4, "On_Off_Value Not Found");
        response_status = ESP_ERR_NOT_FOUND;    //404 not found     (MIGHT NOT WORK)
    }

    device_power = parsedValue;

    // Send the HTTP response with the appropriate status code
    httpd_resp_send(req, NULL, 0);

    return response_status;
}


/* Handler which has to get called when web-client makes requests.*/
httpd_uri_t uri_get_homepage =
{
    .uri = "/",
    .method = HTTP_GET,
    .handler = HomePage_req_handler,
    .user_ctx = NULL
};

httpd_uri_t GetLightValue_FromWebClient =
{
    .uri = "/light_value",
    .method = HTTP_GET,
    .handler = ReceiveLightFrom_Webclient_handler,
    .user_ctx = NULL
};

httpd_uri_t GetOnOff_FromWebClient =
{
    .uri = "/on_off_value",
    .method = HTTP_GET,
    .handler = ReceiveOnOff_Webclient_handler,
    .user_ctx = NULL
};

httpd_handle_t Setup_HTTP_server(void)
{
    httpd_config_t HTTP_ServerConfig = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t HTTP_ServerDetails = NULL;

    if (httpd_start(&HTTP_ServerDetails, &HTTP_ServerConfig) == ESP_OK)
    {
        httpd_register_uri_handler(HTTP_ServerDetails, &uri_get_homepage);
        httpd_register_uri_handler(HTTP_ServerDetails, &GetLightValue_FromWebClient);
        httpd_register_uri_handler(HTTP_ServerDetails, &GetOnOff_FromWebClient);
    }

    return HTTP_ServerDetails;
}

//ADD RESPONSE FROM OTHER/MASTER ON SEND/RECV DATA (SIMPLE ACK or ESP_OK is good enough)
//BUY A ROUTER FOR LATER TESTING/SHOWING
// SHOULD U SEND DATA TO prepai_esp_now() before send_esp in http request???

static esp_err_t espnow_init(void) {
    espnow_send_param_t *send_param;

    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG_subsystem4, "Create mutex fail");
        ESP_LOGE(TAG_subsystem4, "Failure point is espnow_queue");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;

#if IS_MASTER
    memcpy(peer->peer_addr, other_mac_1, ESP_NOW_ETH_ALEN);
#else
    memcpy(peer->peer_addr, master_mac, ESP_NOW_ETH_ALEN);
#endif

    //memcpy(peer->peer_addr, other_mac_1, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG_subsystem4, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG_subsystem4, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, other_mac_1, ESP_NOW_ETH_ALEN);    //Change maybe
    espnow_data_prepare(send_param);

#if IS_MASTER
    //xTaskCreate(&send_task, "send_task", 2048, send_param, 4, NULL);
#else
    xTaskCreate(&receive_task, "receive_task", 2048, NULL, 4, NULL);
#endif

    return ESP_OK;
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG_subsystem4, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG_subsystem4, "Send send queue fail");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG_subsystem4, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG_subsystem4, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG_subsystem4, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->light_level = req_light_level;
    buf->on_off = device_power;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, int32_t *light_level, uint16_t *on_off, int *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    //uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG_subsystem4, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *light_level = buf->light_level;
    *on_off = buf->on_off;
    *magic = buf->magic;
    //crc = buf->crc;
    //buf->crc = 0;
    //crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    //if (crc_cal == crc) {
        //return buf->type;
    //}
    return buf->type;
    //return -1;
}

static void receive_task(void *pvParameter)
{
    espnow_event_t evt;
    int32_t light_level = 0;
    uint16_t on_off = 0;
    int magic = 0;

    while (1) {
        if (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
            switch (evt.id) {
                case ESPNOW_RECV_CB:
                {
                    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                    int ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &light_level, &on_off, &magic);
                    free(recv_cb->data);

                    ESP_LOGI(TAG_subsystem4, "Received data from "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    if(light_level) {
                        ESP_LOGI(TAG_subsystem4, "light_level: %ld", light_level);
                        req_light_level = light_level;
                    }
                    else {
                        ESP_LOGI(TAG_subsystem4, "power_button set to: %s", on_off ? "On" : "Off");
                        device_power = on_off;
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
}

void send_task(void *pvParameters) {
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameters;

    while (1) {
        // Send data using ESP-NOW
        ESP_LOGI(TAG_subsystem4, "about to send");
        espnow_data_prepare(send_param);
        esp_err_t result = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);

        if (result == ESP_OK) {
            ESP_LOGI(TAG_subsystem4, "Data sent successfully");
            ESP_LOGI(TAG_subsystem4, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
        } else {
            ESP_LOGE(TAG_subsystem4, "Failed to send data");
        }
        if (result == ESP_ERR_ESPNOW_NOT_FOUND)
            ESP_LOGI(TAG_subsystem4, "THIS IS THE ERROR");

        // Delay before sending the next data
        vTaskDelay(pdMS_TO_TICKS(5000));  // Adjust the delay as needed     //IS THIS BLOCKING????
    }
}

static void espnow_deinit(espnow_send_param_t *send_param) {
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

// void app_main(void)
// {
//     /* NVS(Non-Volatile Storage) is a partition in flash memory which stores key-value pairs.
//     We can use NVS to store WiFi configuration.*/
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//       ESP_ERROR_CHECK(nvs_flash_erase());
//       ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     ESP_LOGI(TAG_subsystem4, "ESP_WIFI_MODE_STA");
//     WiFi_InitIn_StationMode();

//     espnow_init();

// #if IS_MASTER
//     Setup_HTTP_server();
// #endif
// }
