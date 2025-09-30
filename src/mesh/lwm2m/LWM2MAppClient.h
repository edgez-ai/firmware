#pragma once

#include "configuration.h"

#ifdef ARCH_ESP32

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "examples/client/lwm2mclient.h"
#include "examples/shared/dtlsconnection.h"
#include "examples/shared/tinydtls/dtls_debug.h"
#include "driver/temp_sensor.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include <sys/time.h>

// LWM2M client constants
#define LWM2M_OBJ_ARRAY_SIZE 4
#define LWM2M_RX_BUFFER_SIZE 2048
#define LWM2M_PROTO_BUFFER_SIZE 8000
#define LWM2M_INACTIVITY_LIMIT 40  // seconds
#define LWM2M_WAKEUP_TIME_SEC 20
#define LWM2M_LOCAL_PORT "56830"

// RTC memory variables for deep sleep persistence
extern RTC_DATA_ATTR char rtc_lwm2m_server_uri[128];
extern RTC_DATA_ATTR char rtc_lwm2m_identity[64];  
extern RTC_DATA_ATTR char rtc_lwm2m_psk[17];
extern RTC_DATA_ATTR client_data_t client_data;
extern RTC_FAST_ATTR uint8_t proto_buffer[LWM2M_PROTO_BUFFER_SIZE];

// Global variables
extern float tsens_out;
extern char serialNumber[64];

/**
 * LWM2M Client Manager for Meshtastic
 * 
 * This class manages the LWM2M client lifecycle including:
 * - Client initialization and configuration
 * - Deep sleep persistence of client state
 * - Security object management
 * - Device object management
 * - Network communication handling
 */
class LWM2MClient
{
public:
    static const char *LWM2M_TAG;
    static uint8_t rx_buffer[LWM2M_RX_BUFFER_SIZE];
    static struct timeval sleep_enter_time;
    
    lwm2m_context_t *client_handle = nullptr;
    lwm2m_object_t *objArray[LWM2M_OBJ_ARRAY_SIZE] = {0};
    
    // Helper methods (public for task access)
    esp_err_t readSerialFromFactory(char *serial_out);
    void saveSecurityInfoToRTC(const char *uri, const char *identity, size_t identity_len, 
                              const char *psk, size_t psk_len);
    void parseSerialString(const char *serial, char *serialNumber, char *pinCode, 
                          char *psk_key, char *server);
    esp_err_t resolveServerHostname(const char *server, char *resolved_ip, size_t ip_size);
    void setupObjects(bool isBootstrap, const char *server_uri, const char *identity, 
                     const char *psk, size_t psk_len);
    void handleWakeupReason();
    void checkTemperatureUpdate();
    bool shouldEnterDeepSleep();
    void prepareForDeepSleep();

private:
    bool initialized = false;
    int inactivity_counter = 0;

public:
    LWM2MClient();
    ~LWM2MClient();

    /**
     * Initialize the LWM2M client
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * Run one iteration of the LWM2M client loop
     * Should be called periodically from the main task
     */
    void step();

    /**
     * Start the LWM2M client task
     * This creates a separate FreeRTOS task for the client
     */
    void startClientTask();

    /**
     * Stop the LWM2M client and cleanup resources
     */
    void stop();

    /**
     * Check if client is initialized and ready
     */
    bool isReady() const;

    /**
     * Get current client state
     */
    int getState() const;
};

// C-style function wrappers for LWM2M client callbacks
extern "C" {
    char *security_get_uri(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, 
                          char *uriBuffer, size_t bufferSize);
    char *security_get_public_id(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, 
                                size_t *length);
    char *security_get_secret_key(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, 
                                 size_t *length);
}

// Global LWM2M client instance
extern LWM2MClient *lwm2mClient;

/**
 * Initialize and start the LWM2M client
 * This is the main entry point for LWM2M functionality
 */
void startLwM2MClient();

#endif // ARCH_ESP32