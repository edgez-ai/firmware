#include "LWM2MAppClient.h"
#include "configuration.h"
#include "mesh/MeshService.h" // for service->getLwm2mForPhone()

#ifdef ARCH_ESP32

#include "esp_mac.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>

// Static member definitions
const char* LWM2MClient::LWM2M_TAG = "lwm2m_client";
uint8_t LWM2MClient::rx_buffer[LWM2M_RX_BUFFER_SIZE];
struct timeval LWM2MClient::sleep_enter_time;

// RTC memory variables
RTC_DATA_ATTR char rtc_lwm2m_server_uri[128] = {0};
RTC_DATA_ATTR char rtc_lwm2m_identity[64] = {0};
RTC_DATA_ATTR char rtc_lwm2m_psk[17] = {0};
RTC_DATA_ATTR client_data_t client_data = {0};
RTC_FAST_ATTR uint8_t proto_buffer[LWM2M_PROTO_BUFFER_SIZE];

// Global variables (temperature sensor removed)
char serialNumber[64] = {0};
LWM2MClient *lwm2mClient = nullptr;

// C-style security function prototypes provided by LwM2M stack (implemented in dtlsconnection.c)
extern "C" {
    char *security_get_uri(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId,
                           char *uriBuffer, size_t bufferSize);
    char *security_get_public_id(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId,
                                 size_t *length);
    char *security_get_secret_key(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId,
                                  size_t *length);
}

LWM2MClient::LWM2MClient() : client_handle(nullptr), initialized(false), inactivity_counter(0)
{
    memset(objArray, 0, sizeof(objArray));
}

LWM2MClient::~LWM2MClient()
{
    stop();
}

esp_err_t LWM2MClient::readSerialFromFactory(char *serial_out)
{
    const esp_partition_t *serial_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "serialnumber");
    char serial_buf[256] = {0};
    
    if (serial_part) {
        esp_err_t err = esp_partition_read(serial_part, 0, serial_buf, sizeof(serial_buf) - 1);
        if (err == ESP_OK) {
            // Trim trailing non-printable characters
            int len = sizeof(serial_buf) - 1;
            while (len > 0 && (serial_buf[len-1] < 32 || serial_buf[len-1] > 126)) {
                serial_buf[len-1] = '\0';
                len--;
            }
            printf("[%s] serialnumber: %s\n", LWM2M_TAG, serial_buf);
        } else {
            printf("[%s] Failed to read serialnumber partition: %s\n", LWM2M_TAG, esp_err_to_name(err));
            return ESP_ERR_NOT_FOUND;
        }
        // Ensure null-terminated and copy to output
        serial_buf[sizeof(serial_buf) - 1] = '\0';
        size_t actual_len = strnlen(serial_buf, sizeof(serial_buf));
        memcpy(serial_out, serial_buf, actual_len + 1); // include null terminator
        return ESP_OK;
    } else {
        printf("[%s] Serial partition not found\n", LWM2M_TAG);
        return ESP_ERR_NOT_FOUND;
    }
}

void LWM2MClient::saveSecurityInfoToRTC(const char *uri, const char *identity, size_t identity_len,
                                       const char *psk, size_t psk_len)
{
    if (uri) {
        strncpy(rtc_lwm2m_server_uri, uri, sizeof(rtc_lwm2m_server_uri) - 1);
        rtc_lwm2m_server_uri[sizeof(rtc_lwm2m_server_uri) - 1] = '\0';
    }
    if (identity) {
        memcpy(rtc_lwm2m_identity, identity, (identity_len < sizeof(rtc_lwm2m_identity)) ? identity_len : sizeof(rtc_lwm2m_identity));
    }
    if (psk) {
        memcpy(rtc_lwm2m_psk, psk, (psk_len < sizeof(rtc_lwm2m_psk)) ? psk_len : sizeof(rtc_lwm2m_psk));
    }
}

void LWM2MClient::parseSerialString(const char *serial, char *serialNumber, char *pinCode,
                                   char *psk_key, char *server)
{
    char *token;
    char serial_copy[256];
    strncpy(serial_copy, serial, sizeof(serial_copy) - 1);
    serial_copy[sizeof(serial_copy) - 1] = '\0';
    
    char *rest = serial_copy;
    
    // serialNumber
    token = strtok_r(rest, ":", &rest);
    if (token) strncpy(serialNumber, token, 63);
    // pinCode  
    token = strtok_r(NULL, ":", &rest);
    if (token) strncpy(pinCode, token, 31);
    // psk
    token = strtok_r(NULL, ":", &rest);
    if (token) strncpy(psk_key, token, 63);
    // server
    token = strtok_r(NULL, ":", &rest);
    if (token) strncpy(server, token, 127);
}

esp_err_t LWM2MClient::resolveServerHostname(const char *server, char *resolved_ip, size_t ip_size)
{
    // Check if server is already an IP address
    struct in_addr addr;
    if (inet_aton(server, &addr) != 0) {
        // server is already an IP address
        strncpy(resolved_ip, server, ip_size - 1);
        resolved_ip[ip_size - 1] = '\0';
        return ESP_OK;
    } else {
        // Try to resolve hostname
        struct addrinfo hints = {0};
        struct addrinfo *res = NULL;
        hints.ai_family = AF_INET;
        int err = getaddrinfo(server, NULL, &hints, &res);
        if (err == 0 && res != NULL) {
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)res->ai_addr;
            strncpy(resolved_ip, inet_ntoa(ipv4->sin_addr), ip_size - 1);
            resolved_ip[ip_size - 1] = '\0';
            freeaddrinfo(res);
            return ESP_OK;
        } else {
            printf("[%s] Failed to resolve server hostname: %s\n", LWM2M_TAG, server);
            strncpy(resolved_ip, server, ip_size - 1); // fallback to original
            resolved_ip[ip_size - 1] = '\0';
            return ESP_ERR_NOT_FOUND;
        }
    }
}

void LWM2MClient::setupObjects(bool isBootstrap, const char *server_uri, const char *identity,
                              const char *psk, size_t psk_len)
{
    if (isBootstrap) {
        printf("[%s] Power cycle or other wakeup, performing bootstrap\n", LWM2M_TAG);
        objArray[0] = get_security_object(1, (char*)server_uri, (char*)identity, (char*)psk, psk_len, true);
    } else {
        printf("[%s] Woke from timer, restoring LwM2M config from RTC memory\n", LWM2M_TAG);
        printf("[%s] RTC LwM2M Server URI: %s\n", LWM2M_TAG, rtc_lwm2m_server_uri);
        printf("[%s] RTC LwM2M Identity: %s\n", LWM2M_TAG, rtc_lwm2m_identity);
        printf("[%s] RTC LwM2M PSK Key: %s\n", LWM2M_TAG, rtc_lwm2m_psk);
        objArray[0] = get_security_object(1, rtc_lwm2m_server_uri, rtc_lwm2m_identity, rtc_lwm2m_psk, strlen(rtc_lwm2m_psk), false);
    }

    objArray[1] = get_server_object(1, "U", 300, false);
    objArray[2] = get_object_device();
    objArray[3] = get_test_object();
}

void LWM2MClient::handleWakeupReason()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }
}

// Reused function name to avoid changing call sites; now checks for new LwM2M-related MeshPackets
void LWM2MClient::checkTemperatureUpdate()
{
    // Validate readiness
    if (!client_handle) {
        ESP_LOGW(LWM2M_TAG, "checkTemperatureUpdate: client_handle null");
        return;
    }
    if (!service) {
        ESP_LOGW(LWM2M_TAG, "checkTemperatureUpdate: MeshService 'service' pointer null");
        return;
    }
    if (client_handle->state != STATE_READY) {
        ESP_LOGD(LWM2M_TAG, "checkTemperatureUpdate: state %d not READY, skipping", client_handle->state);
        return; // normal during startup
    }

    // Obtain next LwM2M packet destined for phone (non-blocking dequeue)
    meshtastic_MeshPacket *p = service->getLwm2mForPhone();
    if (!p) {
        ESP_LOGD(LWM2M_TAG, "checkTemperatureUpdate: no new LwM2M packet in queue");
        return; // no new packet
    }

    // Determine payload length (decoded payload if present)
    int new_len = 0;
    if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        new_len = p->decoded.payload.size;
    } else {
        ESP_LOGW(LWM2M_TAG, "checkTemperatureUpdate: unexpected payload_variant=%d", p->which_payload_variant);
    }

    test_data_t *device_data = (test_data_t *)objArray[3]->userData;
    if (device_data && device_data->test_integer != new_len) {
        printf("[%s] LwM2M phone packet length changed from %d -> %d\n", LWM2M_TAG, device_data->test_integer, new_len);
        device_data->test_integer = new_len;
        lwm2m_uri_t uri;
        uri.objectId = 3442; // Test object
        uri.instanceId = 0;
        uri.resourceId = 120; // Test integer resource
        lwm2m_resource_value_changed(client_handle, &uri);
    } else if (device_data) {
        ESP_LOGD(LWM2M_TAG, "checkTemperatureUpdate: packet length %d unchanged", new_len);
    } else {
        ESP_LOGW(LWM2M_TAG, "checkTemperatureUpdate: test_data_t userData missing");
    }

    // Release packet back to pool now that we've consumed it
    service->releaseToPool(p);
}

bool LWM2MClient::shouldEnterDeepSleep()
{
    return (inactivity_counter >= LWM2M_INACTIVITY_LIMIT && 
            client_handle->state == STATE_READY);
}

void LWM2MClient::prepareForDeepSleep()
{
    printf("[%s] No message for %d seconds and LwM2M STATE_READY, shutting off WiFi and going to deep sleep\n", 
            LWM2M_TAG, LWM2M_INACTIVITY_LIMIT);
    //esp_wifi_stop();

    // Save LwM2M config to RTC memory
    printf("[%s] Saving LwM2M config to RTC memory\n", LWM2M_TAG);
    lwm2m_object_t * securityObj = client_data.securityObjP;
    if (securityObj != NULL) {
        char uri_buf[128] = {0};
        char *uri = security_get_uri(client_handle, securityObj, 1, uri_buf, sizeof(uri_buf));
        size_t identity_len = 32;
        char *identity = security_get_public_id(client_handle, securityObj, 1, &identity_len);
        size_t psk_len = 16;
        char *psk = security_get_secret_key(client_handle, securityObj, 1, &psk_len);

        if (uri != NULL) {
            printf("[%s] Security Object URI: %s\n", LWM2M_TAG, uri);
        }
        if (identity != NULL) {
            printf("[%s] Security Object Identity: %s\n", LWM2M_TAG, identity);
        }

        saveSecurityInfoToRTC(uri, identity, identity_len, psk, psk_len);
    }

    printf("Enabling timer wakeup, %ds\n", LWM2M_WAKEUP_TIME_SEC);
    // Deep sleep functionality would be enabled here in actual implementation
    // ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(LWM2M_WAKEUP_TIME_SEC * 1000000));
    // printf("Entering deep sleep\n");
    // gettimeofday(&sleep_enter_time, NULL);
    // esp_deep_sleep_start();

    inactivity_counter = 0; // reset after waking up
}

bool LWM2MClient::initialize()
{
    if (initialized) {
        return true;
    }

    // Temperature sensor logic removed; proceed directly to networking init
    // Initialize networking components
    auto net_res = esp_netif_init();
    if (net_res != ESP_OK) {
        printf("[%s] Failed to initialize netif: %s\n", LWM2M_TAG, esp_err_to_name(net_res));
        return false;
    }
    
    auto event_res = esp_event_loop_create_default();
    if (event_res != ESP_OK && event_res != ESP_ERR_INVALID_STATE) {
        printf("[%s] Failed to create event loop: %s\n", LWM2M_TAG, esp_err_to_name(event_res));
        return false;
    }
    
    // Check for proto buffer data from previous sleep
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && proto_buffer[0] != 0) {
        printf("[%s] LwM2M proto model found in buffer from previous sleep\n", LWM2M_TAG);
    } else {
        printf("[%s] No LwM2M proto model in buffer, starting fresh\n", LWM2M_TAG);
    }
    
    initialized = true;
    return true;
}

void LWM2MClient::step()
{
    if (!client_handle) {
        return;
    }

    time_t tv = lwm2m_gettime();
    lwm2m_step(client_handle, &tv);

    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(client_data.sock, rx_buffer, sizeof(rx_buffer), 0, 
                      (struct sockaddr *)&source_addr, &socklen);

    if (len > 0) {
        printf("[%s] Received %d bytes\n", LWM2M_TAG, len);
        connection_handle_packet(client_data.connList, rx_buffer, len);
        inactivity_counter = 0; // reset inactivity counter
    } else {
        if (client_handle->state == STATE_READY) {
            inactivity_counter++;
            checkTemperatureUpdate();
        }
    }

    //if (shouldEnterDeepSleep()) {
    //    prepareForDeepSleep();
    //} else if (inactivity_counter >= LWM2M_INACTIVITY_LIMIT) {
    //    printf("[%s] No message for %d seconds but LwM2M not ready, skipping sleep\n", 
    //            LWM2M_TAG, LWM2M_INACTIVITY_LIMIT);
    //}
}

// Static task function for FreeRTOS
static void lwm2m_client_task(void *pvParameters)
{
    LWM2MClient *client = static_cast<LWM2MClient*>(pvParameters);
    
    char serial[256] = {0};
    client->readSerialFromFactory(serial);
    
    // Parse serial string into components: serialNumber:PinCode:PSK:Server
    char pinCode[32] = {0};
    char psk_key[64] = {0};
    char server[128] = {0};
    client->parseSerialString(serial, serialNumber, pinCode, psk_key, server);
    
    ESP_LOGI(LWM2MClient::LWM2M_TAG, "serialNumber: %s, pinCode: %s, psk: %s, server: %s", 
            serialNumber, pinCode, psk_key, server);
    
    char LWM2M_SERVER_URI[160] = {0};
    char resolved_ip[64] = {0};
    
    esp_err_t resolve_res = client->resolveServerHostname(server, resolved_ip, sizeof(resolved_ip));
    snprintf(LWM2M_SERVER_URI, sizeof(LWM2M_SERVER_URI), "coaps://%s:5685", resolved_ip);
    ESP_LOGI(LWM2MClient::LWM2M_TAG, "Resolved server hostname: %s %s", resolved_ip, LWM2M_SERVER_URI);
    
    client->handleWakeupReason();
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool isBootstrap = (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER);
    
    if (isBootstrap) {
        char psk_identity[96] = {0};
        snprintf(psk_identity, sizeof(psk_identity), "%s%s", serialNumber, pinCode);
        client->setupObjects(true, LWM2M_SERVER_URI, psk_identity, psk_key, strlen(psk_key));
    } else {
        client->setupObjects(false, nullptr, nullptr, nullptr, 0);
    }

    client_data.sock = create_socket(LWM2M_LOCAL_PORT, client_data.addressFamily);
    if (client_data.sock < 0) {
        ESP_LOGE(LWM2MClient::LWM2M_TAG, "Failed to open socket: %d %s", errno, strerror(errno));
        vTaskDelete(NULL);
        return;
    }
    
    // Set socket to non-blocking mode
    int flags = lwip_fcntl(client_data.sock, F_GETFL, 0);
    lwip_fcntl(client_data.sock, F_SETFL, flags | O_NONBLOCK);
    client_data.securityObjP = client->objArray[0];
    
    lwm2m_context_t *client_handle = lwm2m_init(&client_data);
    if (!client_handle) {
        ESP_LOGE(LWM2MClient::LWM2M_TAG, "Failed to initialize LwM2M client");
        vTaskDelete(NULL);
        return;
    }
    
    client_data.lwm2mH = client_handle;
    client->client_handle = client_handle;
    
    int res = lwm2m_configure(client_handle, serialNumber, NULL, NULL, LWM2M_OBJ_ARRAY_SIZE, client->objArray);
    if (res != 0) {
        ESP_LOGE(LWM2MClient::LWM2M_TAG, "lwm2m_configure failed");
        vTaskDelete(NULL);
        return;
    }
    
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(LWM2MClient::LWM2M_TAG, "Restored LwM2M client state to STATE_READY");
    }
    
    while (1) {
        client->step();
        //lwm2m_object_t * securityObj = client_data.securityObjP;
        //char uri_buf[128] = {0};
        //char *uri = security_get_uri(client_handle, securityObj, 1, uri_buf, sizeof(uri_buf));
        //ESP_LOGI(LWM2MClient::LWM2M_TAG, "Security Object URI: %s", uri ? uri : "NULL");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void LWM2MClient::startClientTask()
{
    if (!initialize()) {
        printf("[%s] Failed to initialize LWM2M client\n", LWM2M_TAG);
        return;
    }
    
    // Create LwM2M client task
    xTaskCreate(lwm2m_client_task, "lwm2m_client", 8192, this, 5, NULL);
}

void LWM2MClient::stop()
{
    if (client_handle) {
        lwm2m_close(client_handle);
        client_handle = nullptr;
    }
    
    if (client_data.sock >= 0) {
        close(client_data.sock);
        client_data.sock = -1;
    }
    
    initialized = false;
}

bool LWM2MClient::isReady() const
{
    return (client_handle != nullptr && client_handle->state == STATE_READY);
}

int LWM2MClient::getState() const
{
    return client_handle ? client_handle->state : STATE_INITIAL;
}

// Global function implementation
void startLwM2MClient()
{
    if (!lwm2mClient) {
        lwm2mClient = new LWM2MClient();
    }
    
    lwm2mClient->startClientTask();
}

#endif // ARCH_ESP32