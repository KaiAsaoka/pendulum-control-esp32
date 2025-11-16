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
    
    PL_Telemetry_ESP32(const char* (&varNames)[N], float* (&pidGainVals)[20])
        : _varNames(varNames),
          _numVars(N)
    {
        for (int i = 0; i < 20; i++) {
            _pidGainVals[i] = pidGainVals[i];
        }
    }

    void begin();
    void sendSnapshot(const float* values, uint64_t timestamp);
    bool pauseTesting();

private:
    void beginSerial();
    void telemetryTask();
    void sendMetadata();
    void sendPID();
    void sendPacket(uint8_t* buffer, size_t size);
    void checkCommands();
    void readGainVals(); 

    const char** _varNames;
    size_t _numVars;
    float* _pidGainVals[20];

    bool _serialStarted = false;
    bool _telemetryStarted = false;
    bool _metadataRequested = false;
    bool _pidSent = false;
    bool _pidReceive = false;
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
