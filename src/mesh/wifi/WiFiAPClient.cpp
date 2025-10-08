#include "configuration.h"
#if HAS_WIFI
// Core includes
#include "NodeDB.h"
#include "RTC.h"
#include "concurrency/Periodic.h"
#include "mesh/wifi/WiFiAPClient.h"
// Application includes
#include "main.h"
#include "mesh/api/WiFiServerAPI.h"
#include "target_specific.h"

#if defined(ESP_PLATFORM)
// Use native ESP-IDF (no Arduino WiFi layer)
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <string.h>
#else
#include <WiFi.h>
#endif

#if HAS_ETHERNET && defined(USE_WS5500)
#include <ETHClass2.h>
#define ETH ETH2
#endif // HAS_ETHERNET

#include <WiFiUdp.h>
#if defined(ESP_PLATFORM)
#if !MESHTASTIC_EXCLUDE_WEBSERVER
#include "mesh/http/WebServer.h"
#endif
#include <ESPmDNS.h> // We still leverage Arduino mDNS component if available; otherwise replace later.
#elif defined(ARCH_RP2040)
#include <SimpleMDNS.h>
#endif

#ifndef DISABLE_NTP
#include "Throttle.h"
#include <NTPClient.h>
#endif

using namespace concurrency;
// LwM2M client task declaration
void startLwM2MClient();

// NTP
WiFiUDP ntpUDP;

#ifndef DISABLE_NTP
NTPClient timeClient(ntpUDP, config.network.ntp_server);
#endif

uint8_t wifiDisconnectReason = 0;

// Stores our hostname
char ourHost[16];

#if defined(ESP_PLATFORM)
// Native ESP-IDF path: state and externs
static bool s_wifi_connected = false;
static esp_netif_t *s_sta_netif = nullptr;
// Externs expected by other modules but not used in native ESP32 path
bool needReconnect = false;
concurrency::Periodic *wifiReconnect = nullptr;
#endif

bool APStartupComplete = 0;
unsigned long lastrun_ntp = 0;

WiFiUDP syslogClient;
Syslog syslog(syslogClient);

#if defined(ARCH_RP2040)
// RP2040 Arduino-style WiFi variables
bool needReconnect = true;
bool isReconnecting = false;
bool wifiReconnectPending = false;
unsigned long wifiReconnectStartMillis = 0;
concurrency::Periodic *wifiReconnect = nullptr;
#endif

#ifdef USE_WS5500
// Startup Ethernet
bool initEthernet()
{
    if ((config.network.eth_enabled) && (ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, ETH_RST_PIN, SPI3_HOST,
                                                   ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN))) {
        WiFi.onEvent(WiFiEvent);
#if !MESHTASTIC_EXCLUDE_WEBSERVER
        createSSLCert(); // For WebServer
#endif
        return true;
    }

    return false;
}
#endif

static void onNetworkConnected()
{
    if (!APStartupComplete) {
        // Start web server
        LOG_INFO("Start network services");

        // start mdns
        if (!MDNS.begin("Meshtastic")) {
            LOG_ERROR("Error setting up mDNS responder!");
        } else {
            LOG_INFO("mDNS Host: Meshtastic.local");
            MDNS.addService("meshtastic", "tcp", SERVER_API_DEFAULT_PORT);
// ESPmDNS (ESP32) and SimpleMDNS (RP2040) have slightly different APIs for adding TXT records
#ifdef ARCH_ESP32
            MDNS.addServiceTxt("meshtastic", "tcp", "shortname", String(owner.short_name));
            MDNS.addServiceTxt("meshtastic", "tcp", "id", String(owner.id));
            // ESP32 prints obtained IP address in WiFiEvent
#elif defined(ARCH_RP2040)
            MDNS.addServiceTxt("meshtastic", "shortname", owner.short_name);
            MDNS.addServiceTxt("meshtastic", "id", owner.id);
            LOG_INFO("Obtained IP address: %s", WiFi.localIP().toString().c_str());
#endif
        }

#ifndef DISABLE_NTP
        LOG_INFO("Start NTP time client");
        timeClient.begin();
        timeClient.setUpdateInterval(60 * 60); // Update once an hour
#endif

        if (config.network.rsyslog_server[0]) {
            LOG_INFO("Start Syslog client");
            // Defaults
            int serverPort = 514;
            const char *serverAddr = config.network.rsyslog_server;
            String server = String(serverAddr);
            int delimIndex = server.indexOf(':');
            if (delimIndex > 0) {
                String port = server.substring(delimIndex + 1, server.length());
                server[delimIndex] = 0;
                serverPort = port.toInt();
                serverAddr = server.c_str();
            }
            syslog.server(serverAddr, serverPort);
            syslog.deviceHostname(getDeviceName());
            syslog.appName("Meshtastic");
            syslog.defaultPriority(LOGLEVEL_USER);
            syslog.enable();
        }

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WEBSERVER
        // Temporarily disabled web server to avoid SSL cert crash
        // if (config.display.displaymode != meshtastic_Config_DisplayConfig_DisplayMode_COLOR) {
        //     initWebServer();
        // }
        LOG_INFO("Web server disabled");
#endif
#if !MESHTASTIC_EXCLUDE_SOCKETAPI
        if (config.display.displaymode != meshtastic_Config_DisplayConfig_DisplayMode_COLOR) {
            initApiServer();
        }
#endif
        APStartupComplete = true;
    }

#if HAS_UDP_MULTICAST
    if (udpHandler && config.network.enabled_protocols & meshtastic_Config_NetworkConfig_ProtocolFlags_UDP_BROADCAST) {
        udpHandler->start();
    }
#endif
}

#if defined(ARCH_RP2040)
static int32_t reconnectWiFi()
{
    const char *wifiName = config.network.wifi_ssid;
    const char *wifiPsw = config.network.wifi_psk;

    if (config.network.wifi_enabled && needReconnect) {

        if (!*wifiPsw) // Treat empty password as no password
            wifiPsw = NULL;

        needReconnect = false;
        isReconnecting = true;

        // Make sure we clear old connection credentials
    WiFi.disconnect(false);
        LOG_INFO("Reconnecting to WiFi access point %s", wifiName);

        // Start the non-blocking wait for 5 seconds
        wifiReconnectStartMillis = millis();
        wifiReconnectPending = true;
        // Do not attempt to connect yet, wait for the next invocation
        return 5000; // Schedule next check soon
    }

    // Check if we are ready to proceed with the WiFi connection after the 5s wait
    if (wifiReconnectPending) {
        if (millis() - wifiReconnectStartMillis >= 5000) {
            if (!WiFi.isConnected()) {
#ifdef CONFIG_IDF_TARGET_ESP32C3
                WiFi.mode(WIFI_MODE_NULL);
                WiFi.useStaticBuffers(true);
                WiFi.mode(WIFI_STA);
#endif
                WiFi.begin(wifiName, wifiPsw);
            }
            isReconnecting = false;
            wifiReconnectPending = false;
        } else {
            // Still waiting for 5s to elapse
            return 100; // Check again soon
        }
    }

#ifndef DISABLE_NTP
    if (WiFi.isConnected() && (!Throttle::isWithinTimespanMs(lastrun_ntp, 43200000) || (lastrun_ntp == 0))) { // every 12 hours
        LOG_DEBUG("Update NTP time from %s", config.network.ntp_server);
        if (timeClient.update()) {
            LOG_DEBUG("NTP Request Success - Setting RTCQualityNTP if needed");

            struct timeval tv;
            tv.tv_sec = timeClient.getEpochTime();
            tv.tv_usec = 0;

            perhapsSetRTC(RTCQualityNTP, &tv);
            lastrun_ntp = millis();
        } else {
            LOG_DEBUG("NTP Update failed");
        }
    }
#endif

    if (config.network.wifi_enabled && !WiFi.isConnected()) {
    needReconnect = APStartupComplete;
        return 1000; // check once per second
    } else {
    onNetworkConnected(); // will only do anything once
        return 300000; // every 5 minutes
    }
}

bool isWifiAvailable()
{

    if (config.network.wifi_enabled && (config.network.wifi_ssid[0])) {
        return true;
#ifdef USE_WS5500
    } else if (config.network.eth_enabled) {
        return true;
#endif
#ifndef ARCH_PORTDUINO
    } else if (WiFi.status() == WL_CONNECTED) {
        // it's likely we have wifi now, but user intends to turn it off in config!
        return true;
#endif
    } else {
        return false;
    }
}

// Disable WiFi
void deinitWifi()
{
    LOG_INFO("WiFi deinit");

    if (isWifiAvailable()) {
#ifdef ARCH_ESP32
        WiFi.disconnect(true, false);
#elif defined(ARCH_RP2040)
        WiFi.disconnect(true);
#endif
        WiFi.mode(WIFI_OFF);
        LOG_INFO("WiFi Turned Off");
        // WiFi.printDiag(Serial);
    }
}

// Startup WiFi
bool initWifi()
{
    if (config.network.wifi_enabled && config.network.wifi_ssid[0]) {

        const char *wifiName = config.network.wifi_ssid;
        const char *wifiPsw = config.network.wifi_psk;

// RP2040 path still can reuse SSL cert creation if available
#if !MESHTASTIC_EXCLUDE_WEBSERVER
    createSSLCert(); // For WebServer
#endif
    WiFi.persistent(false); // Disable flash storage for WiFi credentials
        if (!*wifiPsw) // Treat empty password as no password
            wifiPsw = NULL;

        if (*wifiName) {
            uint8_t dmac[6];
            getMacAddr(dmac);
            snprintf(ourHost, sizeof(ourHost), "Meshtastic-%02x%02x", dmac[4], dmac[5]);

            WiFi.mode(WIFI_STA);
            WiFi.setHostname(ourHost);

            if (config.network.address_mode == meshtastic_Config_NetworkConfig_AddressMode_STATIC &&
                config.network.ipv4_config.ip != 0) {
                // Static IP for RP2040
                WiFi.config(config.network.ipv4_config.ip, config.network.ipv4_config.dns, config.network.ipv4_config.gateway,
                            config.network.ipv4_config.subnet);
            }
            WiFi.setAutoReconnect(true);
            WiFi.setSleep(false);
            LOG_DEBUG("JOINING WIFI soon: ssid=%s", wifiName);
            wifiReconnect = new Periodic("WifiConnect", reconnectWiFi);
        }
        return true;
    } else {
        LOG_INFO("Not using WIFI");
        return false;
    }
}

#endif // ARCH_RP2040 (Arduino style path)

// ESP32 native implementation below
#if defined(ESP_PLATFORM)

static void esp32_on_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    LOG_INFO("Obtained IP address: %s", ip4addr_ntoa((const ip4_addr_t *)&event->ip_info.ip));
    s_wifi_connected = true;
    onNetworkConnected();
    startLwM2MClient();
}

static void esp32_on_wifi_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
    case WIFI_EVENT_STA_START:
        LOG_INFO("WiFi station started");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        LOG_INFO("Connected to access point");
        break;
    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        wifiDisconnectReason = disc->reason;
        LOG_WARN("WiFi disconnected (reason=%d), retrying...", disc->reason);
        s_wifi_connected = false;
        esp_wifi_connect();
        break; }
    default:
        break;
    }
}

bool initWifi()
{
    LOG_DEBUG("initWifi() called - checking config");
    LOG_DEBUG("  wifi_enabled=%d, ssid='%s'", config.network.wifi_enabled, config.network.wifi_ssid);
    
    if (!(config.network.wifi_enabled && config.network.wifi_ssid[0])) {
        LOG_INFO("Not using WIFI (enabled=%d, ssid='%s')", config.network.wifi_enabled, config.network.wifi_ssid);
        return false;
    }

    LOG_INFO("Initializing native ESP-IDF WiFi...");
    
    // Assume Arduino core already initialized NVS; skip manual init

    LOG_DEBUG("Calling esp_netif_init()");
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        LOG_ERROR("esp_netif_init failed %d", err);
        return false;
    }
    LOG_DEBUG("esp_netif_init OK");
    
    static bool loopCreated = false;
    if (!loopCreated) {
        LOG_DEBUG("Creating event loop");
        err = esp_event_loop_create_default();
        if (err != ESP_ERR_INVALID_STATE && err != ESP_OK) {
            LOG_ERROR("event loop create failed %d", err);
            return false;
        }
        LOG_DEBUG("Event loop created/already exists");
        loopCreated = true;
    }
    if (!s_sta_netif) {
        LOG_DEBUG("Creating WiFi STA netif");
        s_sta_netif = esp_netif_create_default_wifi_sta();
    }

    LOG_DEBUG("Initializing WiFi driver");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        LOG_ERROR("esp_wifi_init failed %d", err);
        return false;
    }
    LOG_DEBUG("WiFi driver initialized");

    LOG_DEBUG("Registering event handlers");
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &esp32_on_wifi_event, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &esp32_on_got_ip, NULL, NULL);

    LOG_DEBUG("Configuring WiFi credentials (SSID: %s)", config.network.wifi_ssid);
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, config.network.wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, config.network.wifi_psk, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // Accept WPA2 or better
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    LOG_DEBUG("Setting WiFi mode to STA");
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        LOG_ERROR("esp_wifi_set_mode failed %d", err);
        return false;
    }
    LOG_DEBUG("Setting WiFi config");
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        LOG_ERROR("esp_wifi_set_config failed %d", err);
        return false;
    }
    LOG_DEBUG("Disabling power save mode");
    // Disable power save for performance parity with previous code
    esp_wifi_set_ps(WIFI_PS_NONE);

    LOG_DEBUG("Starting WiFi driver");
    err = esp_wifi_start();
    if (err != ESP_OK) {
        LOG_ERROR("esp_wifi_start failed %d", err);
        return false;
    }

    LOG_INFO("WiFi STA started successfully, connecting to '%s'...", config.network.wifi_ssid);
    LOG_DEBUG("Calling esp_wifi_connect()");
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        LOG_WARN("esp_wifi_connect returned %d (may retry)", err);
    }
    return true; // We'll get events asynchronously
}

void deinitWifi()
{
    LOG_INFO("WiFi deinit");
    esp_wifi_stop();
    esp_wifi_deinit();
}

bool isWifiAvailable()
{
    return s_wifi_connected; // Reflect actual connection state
}
#endif // ESP_PLATFORM

uint8_t getWifiDisconnectReason()
{
    return wifiDisconnectReason;
}
#endif // HAS_WIFI