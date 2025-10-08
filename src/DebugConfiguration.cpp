/* based on https://github.com/arcao/Syslog

MIT License

Copyright (c) 2016 Martin Sloup

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#include "configuration.h"

#include "DebugConfiguration.h"

#if HAS_NETWORKING && defined(ESP_PLATFORM)
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <esp_timer.h>
#include <lwip/ip_addr.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <unistd.h>
#endif

#ifdef ARCH_PORTDUINO
#include "platform/portduino/PortduinoGlue.h"
#endif

/// A C wrapper for LOG_DEBUG that can be used from arduino C libs that don't know about C++ or meshtastic
extern "C" void logLegacy(const char *level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    if (console)
        console->vprintf(level, fmt, args);
    va_end(args);
}

#if HAS_NETWORKING

#if defined(ESP_PLATFORM)

Syslog::Syslog()
    : _socket(-1), _enabled(false), _resolved(false), _port(0), _priDefault(LOGLEVEL_KERN), _priMask(0xff)
{
    ip4_addr_set_zero(&_ip);
    _deviceHostname = SYSLOG_NILVALUE;
    _appName = SYSLOG_NILVALUE;
}

Syslog &Syslog::server(const char *server, uint16_t port)
{
    _server = server ? server : "";
    _port = port;
    _resolved = false;
    return *this;
}

Syslog &Syslog::deviceHostname(const char *deviceHostname)
{
    _deviceHostname = deviceHostname ? deviceHostname : SYSLOG_NILVALUE;
    return *this;
}

Syslog &Syslog::appName(const char *appName)
{
    _appName = appName ? appName : SYSLOG_NILVALUE;
    return *this;
}

Syslog &Syslog::defaultPriority(uint16_t pri)
{
    _priDefault = pri;
    return *this;
}

Syslog &Syslog::logMask(uint8_t priMask)
{
    _priMask = priMask;
    return *this;
}

void Syslog::enable()
{
    if (_enabled)
        return;

    if (_socket >= 0) {
        close(_socket);
        _socket = -1;
    }

    _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (_socket < 0) {
        LOG_ERROR("Syslog socket create failed (%d)", errno);
        return;
    }

    _enabled = true;
}

void Syslog::disable()
{
    if (_socket >= 0) {
        close(_socket);
        _socket = -1;
    }
    _enabled = false;
}

bool Syslog::isEnabled() const
{
    return _enabled;
}

bool Syslog::vlogf(uint16_t pri, const char *fmt, va_list args)
{
    const char *app = _appName.empty() ? SYSLOG_NILVALUE : _appName.c_str();
    return this->vlogf(pri, app, fmt, args);
}

bool Syslog::vlogf(uint16_t pri, const char *appName, const char *fmt, va_list args)
{
    size_t initialLen = strlen(fmt);
    char *message = new char[initialLen + 1];
    size_t len = vsnprintf(message, initialLen + 1, fmt, args);
    if (len > initialLen) {
        delete[] message;
        message = new char[len + 1];
        vsnprintf(message, len + 1, fmt, args);
    }

    bool result = sendLog(pri, appName, message);

    delete[] message;
    return result;
}

bool Syslog::resolve()
{
    if (_resolved)
        return true;

    if (_port == 0 || _server.empty())
        return false;

    ip_addr_t addr;
    if (ipaddr_aton(_server.c_str(), &addr)) {
        ip4_addr_copy(_ip, addr.u_addr.ip4);
        _resolved = true;
        return true;
    }

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo *res = nullptr;
    int ret = getaddrinfo(_server.c_str(), nullptr, &hints, &res);
    if (ret != 0 || res == nullptr) {
        LOG_WARN("Syslog DNS lookup failed for %s (%d)", _server.c_str(), ret);
        if (res)
            freeaddrinfo(res);
        return false;
    }

    auto *addr_in = reinterpret_cast<struct sockaddr_in *>(res->ai_addr);
    ip4_addr_set_u32(&_ip, addr_in->sin_addr.s_addr);
    freeaddrinfo(res);
    _resolved = true;
    return true;
}

bool Syslog::sendLog(uint16_t pri, const char *appName, const char *message)
{
    if (!_enabled || _socket < 0)
        return false;

    if (_port == 0)
        return false;

    if ((LOG_MASK(LOG_PRI(pri)) & _priMask) == 0)
        return true;

    if ((pri & LOG_FACMASK) == 0)
        pri = LOG_MAKEPRI(LOG_FAC(_priDefault), pri);

    if (!resolve())
        return false;

    const char *hostname = _deviceHostname.empty() ? SYSLOG_NILVALUE : _deviceHostname.c_str();
    const char *app = (appName && *appName) ? appName : (_appName.empty() ? SYSLOG_NILVALUE : _appName.c_str());

    std::string payload;
    payload.reserve(strlen(message) + 96);
    payload.push_back('<');
    payload.append(std::to_string(pri));
    payload.append(">1 - ");
    payload.append(hostname);
    payload.push_back(' ');
    payload.append(app);
    payload.append(" - - - ");
    payload.append("\xEF\xBB\xBF");
    payload.push_back('[');
    payload.append(std::to_string(esp_timer_get_time() / 1000000ULL));
    payload.append("]: ");
    payload.append(message);

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(_port);
    dest.sin_addr.s_addr = _ip.addr;

    ssize_t sent = sendto(_socket, payload.data(), payload.size(), 0, reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest));
    if (sent < 0) {
        LOG_WARN("Syslog send failed (%d)", errno);
        return false;
    }
    return static_cast<size_t>(sent) == payload.size();
}

#else

Syslog::Syslog(UDP &client)
{
    this->_client = &client;
    this->_server = NULL;
    this->_port = 0;
    this->_deviceHostname = SYSLOG_NILVALUE;
    this->_appName = SYSLOG_NILVALUE;
    this->_priDefault = LOGLEVEL_KERN;
}

Syslog &Syslog::server(const char *server, uint16_t port)
{
    if (this->_ip.fromString(server)) {
        this->_server = NULL;
    } else {
        this->_server = server;
    }
    this->_port = port;
    return *this;
}

Syslog &Syslog::server(IPAddress ip, uint16_t port)
{
    this->_ip = ip;
    this->_server = NULL;
    this->_port = port;
    return *this;
}

Syslog &Syslog::deviceHostname(const char *deviceHostname)
{
    this->_deviceHostname = (deviceHostname == NULL) ? SYSLOG_NILVALUE : deviceHostname;
    return *this;
}

Syslog &Syslog::appName(const char *appName)
{
    this->_appName = (appName == NULL) ? SYSLOG_NILVALUE : appName;
    return *this;
}

Syslog &Syslog::defaultPriority(uint16_t pri)
{
    this->_priDefault = pri;
    return *this;
}

Syslog &Syslog::logMask(uint8_t priMask)
{
    this->_priMask = priMask;
    return *this;
}

void Syslog::enable()
{
    this->_client->begin(this->_port);
    this->_enabled = true;
}

void Syslog::disable()
{
    this->_enabled = false;
    this->_client->stop();
}

bool Syslog::isEnabled()
{
    return this->_enabled;
}

bool Syslog::vlogf(uint16_t pri, const char *fmt, va_list args)
{
    return this->vlogf(pri, this->_appName, fmt, args);
}

bool Syslog::vlogf(uint16_t pri, const char *appName, const char *fmt, va_list args)
{
    char *message;
    size_t initialLen;
    size_t len;
    bool result;

    initialLen = strlen(fmt);

    message = new char[initialLen + 1];

    len = vsnprintf(message, initialLen + 1, fmt, args);
    if (len > initialLen) {
        delete[] message;
        message = new char[len + 1];

        vsnprintf(message, len + 1, fmt, args);
    }

    result = this->_sendLog(pri, appName, message);

    delete[] message;
    return result;
}

inline bool Syslog::_sendLog(uint16_t pri, const char *appName, const char *message)
{
    int result;
#ifdef ARCH_PORTDUINO
    bool utf = !portduino_config.ascii_logs;
#else
    bool utf = true;
#endif

    if (!this->_enabled)
        return false;

    if ((this->_server == NULL && this->_ip == INADDR_NONE) || this->_port == 0)
        return false;

    // Check priority against priMask values.
    if ((LOG_MASK(LOG_PRI(pri)) & this->_priMask) == 0)
        return true;

    // Set default facility if none specified.
    if ((pri & LOG_FACMASK) == 0)
        pri = LOG_MAKEPRI(LOG_FAC(this->_priDefault), pri);

    if (this->_server != NULL) {
        result = this->_client->beginPacket(this->_server, this->_port);
    } else {
        result = this->_client->beginPacket(this->_ip, this->_port);
    }

    if (result != 1)
        return false;

    this->_client->print('<');
    this->_client->print(pri);
    this->_client->print(F(">1 - "));
    this->_client->print(this->_deviceHostname);
    this->_client->print(' ');
    this->_client->print(appName);
    this->_client->print(F(" - - - "));
    if (utf) {
        this->_client->print(F("\xEF\xBB\xBF"));
    } else {
        this->_client->print(F(" "));
    }
    this->_client->print(F("["));
    this->_client->print(int(millis() / 1000));
    this->_client->print(F("]: "));
    this->_client->print(message);
    this->_client->endPacket();

    return true;
}

#endif // ESP_PLATFORM

#endif // HAS_NETWORKING
