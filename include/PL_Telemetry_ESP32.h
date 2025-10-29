#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class PL_Telemetry_ESP32 {
public:
    struct Snapshot {
        float* vars[64];              // allocated based on numVars
        uint64_t timestamp_us;
    };

    struct __attribute__((packed)) TelemetryPacketHeader {
        uint16_t sync;
        uint16_t seq;
        uint8_t num_snapshots;
        uint8_t num_vars;
    };

    template<size_t N>
    PL_Telemetry_ESP32(const char* ssid,
                        const char* password,
                        const IPAddress& localIP,
                        const IPAddress& pcIP,
                        unsigned int udpPort,
                        const char* (&varNames)[N])
        : _ssid(ssid),
        _password(password),
        _localIP(localIP),
        _pcIP(pcIP),
        _udpPort(udpPort),
        _varNames(varNames),   // decay to const char* const*
        _numVars(N) {}

    void begin();
    void sendSnapshot(const float* values, uint64_t timestamp);

private:
    void wifiBegin();
    void serialBegin();
    void telemetryTask();
    void sendMetadata();
    void sendPacket(uint8_t* buffer, size_t size);
    void checkCommands();

    const char* _ssid;
    const char* _password;
    IPAddress _localIP;
    IPAddress _pcIP;
    unsigned int _udpPort;
    const char** _varNames;
    size_t _numVars;

    WiFiUDP _udp;
    bool _wifiStarted = false;
    bool _serialStarted = false;
    bool _telemetryStarted = false;
    bool _metadataRequested = false;
    unsigned long _lastPulseTime = 0;
    uint16_t _packetSeq = 0;

    static const uint8_t _BATCH_SIZE = 50;
    static const unsigned long _PULSE_TIMEOUT = 2000; // ms

    struct InternalSnapshot {
        float vars[64];   // max supported vars
        uint64_t timestamp_us;
    };

    QueueHandle_t _snapshotQueue;
};
