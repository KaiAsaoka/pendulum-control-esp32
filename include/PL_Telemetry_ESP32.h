#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PID.h>

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
    
    PL_Telemetry_ESP32(const char* (&varNames)[N], pidParams setAngleXParams, pidParams setAngleYParams,
                        pidParams setPWMXParams, pidParams setPWMYParams)
        : _varNames(varNames),
          _numVars(N),
          setAngleXParams(setAngleXParams),
          setAngleYParams(setAngleYParams),
          setPWMXParams(setPWMXParams),
          setPWMYParams(setPWMYParams)
    {}

    void begin();
    void sendSnapshot(const float* values, uint64_t timestamp);
    bool updateGainVals();
    bool pauseTesting();

    pidParams setAngleXParams;
    pidParams setAngleYParams;
    pidParams setPWMXParams;
    pidParams setPWMYParams;

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
    
    std::array<pidParams*, 4> _pidParams = {&setAngleXParams, &setAngleYParams, &setPWMXParams, &setPWMYParams};

    bool _serialStarted = false;
    bool _telemetryStarted = false;
    bool _metadataRequested = false;
    bool _pidSent = false;
    bool _pidReceive = false;
    unsigned long _lastPulseTime = 0;
    uint16_t _packetSeq = 0;

    static const uint8_t _BATCH_SIZE = 1;
    static const unsigned long _PULSE_TIMEOUT = 2000; // ms

    struct InternalSnapshot {
        float vars[64];   // max supported vars
        uint64_t timestamp_us;
    };

    QueueHandle_t _snapshotQueue;
};
