#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <../common/Data_type.h>
#include <lwip/netdb.h>

#include "drivers\WIFI\OtaServer.h"
#include "drivers\WIFI\wifi_esp32.h"
#include "queuemonitor.h"

#define DEBUG_MODULE "WIFI_UDP"
static const char* TAG = DEBUG_MODULE;
#include "debug_cf.h"

#define UDP_SERVER_PORT 2390
#define UDP_SEND_PORT 2391
#define UDP_SERVER_BUFSIZE 128

static struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
//#define WIFI_SSID      "Udp Server"
static char WIFI_SSID[32] = "ESP-DRONE_BCDDC2D254AD";
static char WIFI_PWD[64] = "12345678"; // 123456789

// static char WIFI_SSID_Station[32] = "DESKTOP-0KDNH8C";
static char WIFI_SSID_Station[32] = "Pretty fly for a wifi";
static char WIFI_PWD_Station[64] = "JRMinor1!";
#pragma GCC diagnostic pop

#define MAX_STA_CONN (1)

// #define STATION_MODE
#define FUCK
// #define DP

static char rx_buffer[UDP_SERVER_BUFSIZE];
static char tx_buffer[UDP_SERVER_BUFSIZE];
const int addr_family = (int)AF_INET;
const int ip_protocol = IPPROTO_IP;
static struct sockaddr_in dest_addr;
static int sock;

static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;
static UDPPacket inPacket;
static UDPPacket outPacket;

static xSemaphoreHandle s_semph_get_ip_addrs;
static esp_netif_t* s_example_esp_netif = NULL;

static WifiServerType RunningServer;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;

static esp_err_t udp_server_create(void* arg);

#define esp_ip4_get_byte(ipaddr, idx) (((const uint8_t*)(&(ipaddr)))[idx])
#define esp_ip41(ipaddr) esp_ip4_get_byte(ipaddr, 0)
#define esp_ip42(ipaddr) esp_ip4_get_byte(ipaddr, 1)
#define esp_ip43(ipaddr) esp_ip4_get_byte(ipaddr, 2)
#define esp_ip44(ipaddr) esp_ip4_get_byte(ipaddr, 3)

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    DEBUG_PRINT_LOCAL("STA Event %d", event_id);
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
        DEBUG_PRINT_LOCAL("station " MACSTR " join, AID=%d",
            MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
        DEBUG_PRINT_LOCAL("station " MACSTR " leave, AID=%d",
            MAC2STR(event->mac), event->aid);
    }
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        DEBUG_PRINT_LOCAL("Got Ip From AP " IPSTR "join", esp_ip41(event->ip_info.ip.addr), esp_ip42(event->ip_info.ip.addr), esp_ip43(event->ip_info.ip.addr), esp_ip44(event->ip_info.ip.addr));
        DEBUG_PRINT_LOCAL("Got Ip From AP %d join", event->ip_info.ip.addr);

    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
        // DEBUG_PRINT_LOCAL("station  leave, AID=%d");
    }
}

bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket* in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t* data)
{
    static UDPPacket outStage;
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    //?DEBUG_PRINTI("xQueueSend wifiSendData");
    return (xQueueSend(udpDataTx, &outStage, 100) == pdTRUE);
};

static esp_err_t udp_server_create(void* arg)
{
    if (isUDPInit) {
        return ESP_OK;
    }

    struct sockaddr_in* pdest_addr = &dest_addr;
    pdest_addr->sin_addr.s_addr = htonl(INADDR_ANY);
    pdest_addr->sin_family = AF_INET;
    pdest_addr->sin_port = htons(UDP_SERVER_PORT);

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    DEBUG_PRINT_LOCAL("Socket created");

    int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
    }
    DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

    isUDPInit = true;
    return ESP_OK;
}

static void udp_server_rx_task(void* pvParameters)
{
    // uint8_t cksum = 0;
    socklen_t socklen = sizeof(source_addr);

    while (true) {
        if (isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);
        /* command step - receive  01 from Wi-Fi UDP */
        if (len < 0) {
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            break;
        } else if (len > WIFI_RX_TX_PACKET_SIZE - 4) {
            DEBUG_PRINT_LOCAL("Received data length = %d > 64", len);
        } else {
            // copy part of the UDP packet
            //  DEBUG_PRINTI("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            //               inPacket.data[0], inPacket.data[1], inPacket.data[2], inPacket.data[3], inPacket.data[4], inPacket.data[5], inPacket.data[6], inPacket.data[7], inPacket.data[8], inPacket.data[9],
            //               inPacket.data[10], inPacket.data[11], inPacket.data[12], inPacket.data[13], inPacket.data[14], inPacket.data[15], inPacket.data[16], inPacket.data[17], inPacket.data[18], inPacket.data[19],
            //               inPacket.data[20], inPacket.data[21], inPacket.data[22], inPacket.data[23], inPacket.data[24], inPacket.data[25], inPacket.data[26], inPacket.data[27], inPacket.data[28], inPacket.data[29], inPacket.data[30],
            //               inPacket.data[31], inPacket.data[31], inPacket.data[32], inPacket.data[33], inPacket.data[34], inPacket.data[35], inPacket.data[36], inPacket.data[37], inPacket.data[38], inPacket.data[39]);

            // uint8_t cksumPree = rx_buffer[len];
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
            memcpy(inPacket.data, rx_buffer, len);
            uint8_t cksum = inPacket.data[len - 1];
            uint8_t cksumCalk = calculate_cksum(inPacket.data, len - 1);
            // remove cksum, do not belong to CRTP
            inPacket.size = len - 1;
            // check packet
            if (cksum == cksumCalk && inPacket.size < 64) {

                // DEBUG_PRINTI("xQueueSend udp_server_rx_task %d,%d,%d,%d,%d,%d", inPacket.data[0], inPacket.data[0], inPacket.data[0], inPacket.data[0], inPacket.data[0], inPacket.data[0]);
                xQueueSend(udpDataRx, &inPacket, 2);
                if (!isUDPConnected)
                    isUDPConnected = true;
            } else {
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched c1:%d, c2:%d", cksum, cksumCalk);
            }

#ifdef DEBUG_UDP
            DEBUG_PRINT_LOCAL("1.Received data size = %d  %02X \n cksum = %02X", len, inPacket.data[0], cksum);
            for (size_t i = 0; i < len; i++) {
                DEBUG_PRINT_LOCAL(" data[%d] = %02X ", i, inPacket.data[i]);
            }
#endif
        }
    }
}

static void udp_server_tx_task(void* pvParameters)
{

    while (true) {
        if (isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        if ((xQueueReceive(udpDataTx, &outPacket, 5) == pdTRUE) && isUDPConnected) {
            memcpy(tx_buffer, outPacket.data, outPacket.size);
            tx_buffer[outPacket.size] = calculate_cksum((unsigned char*)tx_buffer, outPacket.size);
            tx_buffer[outPacket.size + 1] = 0;

            int err = sendto(sock, tx_buffer, outPacket.size + 1, 0, (struct sockaddr*)&source_addr, sizeof(source_addr));
            if (err < 0) {
                DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
                continue;
            }
            //#ifdef DEBUG_UDP
            char* buf;
            buf = (char*)malloc(300); // 3 * 64  with som extra
            // memset(buf, 0, 300);
            for (size_t i = 0; i < outPacket.size + 1; i++) {
                sprintf(buf + i * 3, "%02X,", tx_buffer[i]);
            }
            DEBUG_PRINT_LOCAL(" data_send_Wifi size:%d , %s ", outPacket.size, buf);
            free(buf);
            //#endif
        }
    }
}

/**
 * @brief Checks the netif description if it contains specified prefix.
 * All netifs created withing common connect component are prefixed with the module TAG,
 * so it returns true if the specified netif is owned by this module
 */
// static bool is_our_netif(const char* prefix, esp_netif_t* netif)
// {
//     return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
// }

static esp_ip4_addr_t s_ip_addr;

static void on_got_ip(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    DEBUG_PRINT_LOCAL("Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    memcpy(&s_ip_addr, &event->ip_info.ip, sizeof(s_ip_addr));
    xSemaphoreGive(s_semph_get_ip_addrs);
}

static void on_wifi_disconnect(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    DEBUG_PRINT_LOCAL("Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

static esp_netif_t* wifi_start(void)
{
    char* desc;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Prefix the interface description with the module TAG
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    asprintf(&desc, "%s: %s", TAG, esp_netif_config.if_desc);
    esp_netif_config.if_desc = desc;
    esp_netif_config.route_prio = 128;
    esp_netif_t* netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    free(desc);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_EXAMPLE_WIFI_SSID,
            .password = CONFIG_EXAMPLE_WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .btm_enabled = 0,
        },
    };
    DEBUG_PRINT_LOCAL("Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    s_semph_get_ip_addrs = xSemaphoreCreateCounting(1, 0);
    return netif;
}

static void wifi_stop(void)
{
    // esp_netif_t* wifi_netif = get_example_netif_from_desc("sta");
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));

    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_example_esp_netif));
    esp_netif_destroy(s_example_esp_netif);
    s_example_esp_netif = NULL;
}

esp_err_t WIFI_connect(void)
{
    if (s_semph_get_ip_addrs != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    wifi_start();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&wifi_stop));
    DEBUG_PRINT_LOCAL("Waiting for IP(s)");
    for (int i = 0; i < 1; ++i) {
        xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
    }
    // iterate over active interfaces, and print out IPs of "our" netifs
    esp_netif_t* netif = NULL;
    esp_netif_ip_info_t ip;
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        DEBUG_PRINT_LOCAL("Connected to %s", esp_netif_get_desc(netif));
        ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));

        DEBUG_PRINT_LOCAL("- IPv4 address: " IPSTR, IP2STR(&ip.ip));
    }
    return ESP_OK;
}

void wifiInit(WifiServerType ServerType)
{
    if (isInit) {
        return;
    }
    RunningServer = ServerType;
#ifdef DB
    if (ServerType == OTA) {
        Start_OTA_Wifi_Server();
    }
#else
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifdef FUCK
    ESP_ERROR_CHECK(WIFI_connect());
#else

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config;

#ifdef STATION_MODE
    esp_netif_t* sta_netif = NULL;
    sta_netif = esp_netif_create_default_wifi_sta();

    memcpy(wifi_config.sta.ssid, WIFI_SSID_Station, strlen(WIFI_SSID_Station) );
    memcpy(wifi_config.sta.password, WIFI_PWD_Station, strlen(WIFI_PWD_Station) );
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.channel = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
#else
    esp_netif_t* ap_netif = NULL;
    uint8_t mac[6];
    ap_netif = esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    // sprintf(WIFI_SSID, "ESP-DRONE_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID));
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD));
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.channel = 6;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
#endif
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr = ipaddr_addr("192.168.43.42"),
    };

#ifdef STATION_MODE
    // ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));
    // ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
    // ESP_ERROR_CHECK(esp_netif_dhcpc_start(sta_netif));
     ESP_ERROR_CHECK(esp_wifi_connect());

    DEBUG_PRINT_LOCAL("wifi_init_station complete.SSID:%s", WIFI_SSID_Station);
#else
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    DEBUG_PRINT_LOCAL("wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);
#endif
#endif
        if (ServerType == CommsLink) {
        // This should probably be reduced to a CRTP packet size
        udpDataRx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
        DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
        udpDataTx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
        DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);
        if (udp_server_create(NULL) == ESP_FAIL) {
            DEBUG_PRINT_LOCAL("UDP server create socket failed!!!");
        } else {
            DEBUG_PRINT_LOCAL("UDP server create socket succeed!!!");
        }
        xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
        xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
    }
    if (ServerType == OTA) {
        Start_OTA_Wifi_Server();
    }
#endif
    isInit = true;
}