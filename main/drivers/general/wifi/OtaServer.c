
#include "drivers\WIFI\OtaServer.h"
#include "esp_eth.h"
#include "esp_flash_partitions.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_tls_crypto.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "string.h"
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>
#include <sys/param.h>

#define DEBUG_MODULE "WIFI_OTA"
#include "debug_cf.h"

typedef struct
{
    const char* username;
    const char* password;
} basic_auth_info_t;

#define HTTPD_401 "401 UNAUTHORIZED" /*!< HTTP Response 401 */

static basic_auth_info_t auth_info = {
    .username = "ota",
    .password = "12345",
};

static void wifi_task(void* Param);
static void ota_task(void* Param);
static httpd_handle_t start_webserver(void);
char* SerialData;

void Start_OTA_Wifi_Server()
{
    DEBUG_PRINTI("OTA Bitch!\n");

    start_webserver();
    DEBUG_PRINTI("start_webserver");

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // Validate image some how, then call:
            esp_ota_mark_app_valid_cancel_rollback();
            // If needed: esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }

    //Put all the wifi stuff in a separate task so that we don't have to wait for a connection
    // xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 0, NULL);
    // xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
}

//-----------------------------------------------------------------------------
static char* http_auth_basic(const char* username, const char* password)
{
    int out;
    char user_info[128];
    static char digest[512];
    size_t n = 0;
    sprintf(user_info, "%s:%s", username, password);

    esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char*)user_info, strlen(user_info));

    // 6: The length of the "Basic " string
    // n: Number of bytes for a base64 encode format
    // 1: Number of bytes for a reserved which be used to fill zero
    if (sizeof(digest) > (6 + n + 1)) {
        strcpy(digest, "Basic ");
        esp_crypto_base64_encode((unsigned char*)digest + 6, n, (size_t*)&out, (const unsigned char*)user_info, strlen(user_info));
    }

    return digest;
}

//-----------------------------------------------------------------------------
char auth_buffer[512];
const char html_post_file[] = "\
<style>\n\
.progress {margin: 15px auto;  max-width: 500px;height: 30px;}\n\
.progress .progress__bar {\n\
  height: 100%; width: 1%; border-radius: 15px;\n\
  background: repeating-linear-gradient(135deg,#336ffc,#036ffc 15px,#1163cf 15px,#1163cf 30px); }\n\
 .status {font-weight: bold; font-size: 30px;};\n\
</style>\n\
<link rel=\"stylesheet\" href=\"https://cdnjs.cloudflare.com/ajax/libs/twitter-bootstrap/2.2.1/css/bootstrap.min.css\">\n\
<div class=\"well\" style=\"text-align: center;\">\n\
  <div class=\"btn\" onclick=\"file_sel.click();\"><i class=\"icon-upload\" style=\"padding-right: 5px;\"></i>Upload Firmware</div>\n\
  <div class=\"progress\"><div class=\"progress__bar\" id=\"progress\"></div></div>\n\
  <div class=\"status\" id=\"status_div\"></div>\n\
</div>\n\
<input type=\"file\" id=\"file_sel\" onchange=\"upload_file()\" style=\"display: none;\">\n\
<script>\n\
function upload_file() {\n\
  document.getElementById(\"status_div\").innerHTML = \"Upload in progress\";\n\
  let data = document.getElementById(\"file_sel\").files[0];\n\
  xhr = new XMLHttpRequest();\n\
  xhr.open(\"POST\", \"/ota\", true);\n\
  xhr.setRequestHeader('X-Requested-With', 'XMLHttpRequest');\n\
  xhr.upload.addEventListener(\"progress\", function (event) {\n\
     if (event.lengthComputable) {\n\
    	 document.getElementById(\"progress\").style.width = (event.loaded / event.total) * 100 + \"%\";\n\
     }\n\
  });\n\
  xhr.onreadystatechange = function () {\n\
    if(xhr.readyState === XMLHttpRequest.DONE) {\n\
      var status = xhr.status;\n\
      if (status >= 200 && status < 400)\n\
      {\n\
        document.getElementById(\"status_div\").innerHTML = \"Upload accepted. Device will reboot.\";\n\
      } else {\n\
        document.getElementById(\"status_div\").innerHTML = \"Upload rejected!\";\n\
      }\n\
    }\n\
  };\n\
  xhr.send(data);\n\
  return false;\n\
}\n\
</script>";

//-----------------------------------------------------------------------------
static esp_err_t basic_auth_get_handler(httpd_req_t* req)
{
    basic_auth_info_t* basic_auth_info = (basic_auth_info_t*)req->user_ctx;

    size_t buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if ((buf_len > 1) && (buf_len <= sizeof(auth_buffer))) {
        if (httpd_req_get_hdr_value_str(req, "Authorization", auth_buffer, buf_len) == ESP_OK) {
            char* auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
            if (!strncmp(auth_credentials, auth_buffer, buf_len)) {
                printf("Authenticated!\n");
                httpd_resp_set_status(req, HTTPD_200);
                httpd_resp_set_hdr(req, "Connection", "keep-alive");
                httpd_resp_send(req, html_post_file, strlen(html_post_file));
                return ESP_OK;
            } else {
                printf(auth_credentials);
                printf("\n");
                printf(auth_buffer);
                printf("\n");
            }
        }
    }
    //   printf("Authenticated!\n");
    //   httpd_resp_set_status(req, HTTPD_200);
    //   httpd_resp_set_hdr(req, "Connection", "keep-alive");
    //   httpd_resp_send(req, html_post_file, strlen(html_post_file));
    //   return ESP_OK;

    DEBUG_PRINTI("Not authenticated\n");
    httpd_resp_set_status(req, HTTPD_401);
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}

extern const char html_file_array[] asm("_binary_index_html_start");

static esp_err_t Index_handler(httpd_req_t* req)
{
    DEBUG_PRINTI("indexLen =%d \n", strlen(html_file_array));

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_send(req, html_file_array, strlen(html_file_array));
    return ESP_OK;
}

static esp_err_t Data_handler(httpd_req_t* req)
{
    DEBUG_PRINTI("indexLen =%d \n", strlen(SerialData));
    char dat[512];
    sprintf(dat, "%f, %f, %f", 1.0, 1.0, 1.0);

    httpd_resp_set_status(req, HTTPD_200);
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_send(req, SerialData, strlen(SerialData));

    return ESP_OK;
}

static esp_err_t return_failure(esp_ota_handle_t update_handle, httpd_req_t* req)
{
    if (update_handle) {
        esp_ota_abort(update_handle);
    }

    httpd_resp_set_status(req, HTTPD_500); // Assume failure
    httpd_resp_send(req, NULL, 0);
    return ESP_FAIL;
}

//-----------------------------------------------------------------------------
static esp_err_t ota_post_handler(httpd_req_t* req)
{
    char buf[256];
    httpd_resp_set_status(req, HTTPD_500); // Assume failure

    int ret, remaining = req->content_len;
    DEBUG_PRINTI("Receiving\n");

    esp_ota_handle_t update_handle = 0;
    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(NULL);
    const esp_partition_t* running = esp_ota_get_running_partition();

    if (update_partition == NULL) {
        DEBUG_PRINTI("Uh oh, bad things\n");
        return return_failure(update_handle, req);
    }

    DEBUG_PRINTI("Writing partition: type %d, subtype %d, offset 0x%08x\n", update_partition->type, update_partition->subtype, update_partition->address);
    DEBUG_PRINTI("Running partition: type %d, subtype %d, offset 0x%08x\n", running->type, running->subtype, running->address);
    esp_err_t err = ESP_OK;
    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        DEBUG_PRINTI("esp_ota_begin failed (%s)", esp_err_to_name(err));
        return return_failure(update_handle, req);
    }
    while (remaining > 0) {
        // Read the data for the request
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                // Retry receiving if timeout occurred
                continue;
            }

            return return_failure(update_handle, req);
        }

        size_t bytes_read = ret;

        remaining -= bytes_read;
        err = esp_ota_write(update_handle, buf, bytes_read);
        if (err != ESP_OK) {
            return return_failure(update_handle, req);
        }
    }

    DEBUG_PRINTI("Receiving done\n");

    // End response
    if ((esp_ota_end(update_handle) == ESP_OK) && (esp_ota_set_boot_partition(update_partition) == ESP_OK)) {
        DEBUG_PRINTI("OTA Success?!\n Rebooting\n");
        fflush(stdout);

        httpd_resp_set_status(req, HTTPD_200);
        httpd_resp_send(req, NULL, 0);

        vTaskDelay(2000 / portTICK_RATE_MS);
        esp_restart();

        return ESP_OK;
    }
    DEBUG_PRINTI("OTA End failed (%s)!\n", esp_err_to_name(err));
    return return_failure(update_handle, req);
}

//-----------------------------------------------------------------------------
esp_err_t http_404_error_handler(httpd_req_t* req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        return ESP_OK; // Return ESP_OK to keep underlying socket open
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        return ESP_FAIL; // Return ESP_FAIL to close underlying socket
    }

    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "404 error");
    return ESP_FAIL; // For any other URI send 404 and close socket
}

//-----------------------------------------------------------------------------
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    DEBUG_PRINTI("Starting server on port %d\n", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        static const httpd_uri_t ota = {
            .uri = "/ota",
            .method = HTTP_POST,
            .handler = ota_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &ota);

        static httpd_uri_t root = {
            .uri = "/ota",
            .method = HTTP_GET,
            .handler = basic_auth_get_handler,
            .user_ctx = &auth_info,
        };
        httpd_register_uri_handler(server, &root);

        static httpd_uri_t indexX = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = Index_handler,
            .user_ctx = NULL,
        };
        httpd_register_uri_handler(server, &indexX);

        static httpd_uri_t serialData = {
            .uri = "/serialData",
            .method = HTTP_GET,
            .handler = Data_handler,
            .user_ctx = NULL,
        };
        httpd_register_uri_handler(server, &serialData);
    }

    return NULL;
}

//*
//-----------------------------------------------------------------------------
static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*)arg;

    if (*server) {
        DEBUG_PRINTI("Stopping webserver");
        httpd_stop(*server);
        *server = NULL;
    }
}

//-----------------------------------------------------------------------------
static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    DEBUG_PRINTI("Connected!\n");
    httpd_handle_t* server = (httpd_handle_t*)arg;

    if (*server == NULL) {
        DEBUG_PRINTI("Starting webserver");
        *server = start_webserver();
    }
}

//-----------------------------------------------------------------------------
static void wifi_task(void* Param)
{
    DEBUG_PRINTI("Wifi task starting");

    httpd_handle_t server = NULL;

    esp_netif_init();
    DEBUG_PRINTI("esp_netif_init");
    esp_event_loop_create_default();
    DEBUG_PRINTI("esp_event_loop_create_default");
    example_connect();
    DEBUG_PRINTI("example_connect");

    // Register event handlers to stop the server when Wi-Fi is disconnected, and re-start it upon connection
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server);

    // server =
    start_webserver();
    DEBUG_PRINTI("start_webserver");

    const uint32_t task_delay_ms = 1000;
    while (1) {
        vTaskDelay(task_delay_ms / portTICK_RATE_MS);
    }
}

//-----------------------------------------------------------------------------
static void ota_task(void* Param)
{
    //  #define HASH_LEN 32         // SHA-256 digest length
    //  uint8_t sha_256[HASH_LEN] = { 0 };
    //  esp_partition_t partition;
    //
    //  partition.address   = ESP_PARTITION_TABLE_OFFSET;
    //  partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
    //  partition.type      = ESP_PARTITION_TYPE_DATA;
    //  esp_partition_get_sha256(&partition, sha_256);
    //
    //  partition.address   = ESP_BOOTLOADER_OFFSET;
    //  partition.size      = ESP_PARTITION_TABLE_OFFSET;
    //  partition.type      = ESP_PARTITION_TYPE_APP;
    //  esp_partition_get_sha256(&partition, sha_256);
    //
    //  esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // Validate image some how, then call:
            esp_ota_mark_app_valid_cancel_rollback();
            // If needed: esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }

    const uint32_t task_delay_ms = 10;
    while (1) {
        vTaskDelay(task_delay_ms / portTICK_RATE_MS);
    }
}
//*/