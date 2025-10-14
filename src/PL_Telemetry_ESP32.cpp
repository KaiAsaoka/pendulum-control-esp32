#include "PL_Telemetry_ESP32.h"

void PL_Telemetry_ESP32::wifiBegin() {
    IPAddress gateway(192,168,137,1);
    IPAddress subnet(255,255,255,0);

    WiFi.config(_localIP, gateway, subnet);
    WiFi.begin(_ssid, _password);

    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        // Serial.print(".");
    }
    // Serial.println("\nWi-Fi connected! IP: " + WiFi.localIP().toString());

    _udp.begin(_udpPort);
}

void PL_Telemetry_ESP32::sendMetadata() {
    uint8_t buffer[256];
    size_t offset = 0;

    buffer[offset++] = 0xCD;
    buffer[offset++] = 0xAB;
    buffer[offset++] = _numVars;

    for(size_t i=0;i<_numVars;i++) {
        uint8_t len = strlen(_varNames[i]);
        buffer[offset++] = len;
        memcpy(buffer+offset, _varNames[i], len);
        offset += len;
    }

    _udp.beginPacket(_pcIP, _udpPort);
    _udp.write(buffer, offset);
    _udp.endPacket();
}

void PL_Telemetry_ESP32::checkCommands() {
    int packetSize = _udp.parsePacket();
    if(packetSize) {
        char buf[16];
        int len = _udp.read(buf, sizeof(buf)-1);
        buf[len] = 0;

        if(strcmp(buf,"METADATA") == 0) {
            _metadataRequested = true;
        }
        else if(strcmp(buf,"START") == 0) {
            _telemetryStarted = true;
            _lastPulseTime = millis();
            // Serial.println("Telemetry started!");
        }
        else if(strcmp(buf,"PULSE") == 0) {
            _lastPulseTime = millis();
            // Serial.println("Pulse received");
        }
    }
}


void PL_Telemetry_ESP32::telemetryTask() {
    // Use internal snapshot array
    InternalSnapshot batch[_BATCH_SIZE];
    wifiBegin(); // initialize Wi-Fi and UDP

    for (;;) {
        checkCommands();

        // If telemetry started but no pulse received within timeout, reset
        if (_telemetryStarted && (millis() - _lastPulseTime > _PULSE_TIMEOUT)) {
            // Serial.println("Keepalive lost! Returning to metadata mode.");
            _telemetryStarted = false;
            _metadataRequested = false;
        }

        // If metadata requested but telemetry not started, keep sending
        if (_metadataRequested && !_telemetryStarted) {
            sendMetadata();
            vTaskDelay(pdMS_TO_TICKS(10)); // send every 10ms until START
            continue;
        }

        // Collect snapshots from queue
        uint8_t count = 0;
        while (count < _BATCH_SIZE) {
            if (xQueueReceive(_snapshotQueue, &batch[count], 0) == pdPASS) {
                count++;
            } else {
                break;
            }
        }

        if (count == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Build telemetry packet
        size_t packetSize = sizeof(TelemetryPacketHeader) + count * (sizeof(float) * _numVars + sizeof(uint64_t)) + 2;
        uint8_t* buffer = new uint8_t[packetSize];

        TelemetryPacketHeader* header = (TelemetryPacketHeader*)buffer;
        header->sync = 0xAA55;
        header->seq = _packetSeq++;
        header->num_snapshots = count;
        header->num_vars = _numVars;

        uint8_t* ptr = buffer + sizeof(TelemetryPacketHeader);
        for (uint8_t i = 0; i < count; i++) {
            // Copy floats first, timestamp last (matches old GUI)
            memcpy(ptr, batch[i].vars, _numVars * sizeof(float));
            ptr += _numVars * sizeof(float);
            memcpy(ptr, &batch[i].timestamp_us, sizeof(uint64_t));
            ptr += sizeof(uint64_t);
        }

        // CRC placeholder
        uint16_t* crcPtr = (uint16_t*)(buffer + packetSize - 2);
        *crcPtr = 0;

        // Send UDP packet
        _udp.beginPacket(_pcIP, _udpPort);
        _udp.write(buffer, packetSize);
        _udp.endPacket();

        delete[] buffer;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void PL_Telemetry_ESP32::begin() {
    _snapshotQueue = xQueueCreate(200,sizeof(InternalSnapshot));

    xTaskCreatePinnedToCore(
        [](void* arg) {
            ((PL_Telemetry_ESP32*)arg)->telemetryTask();
        },
        "TelemetryTask",
        16382,
        this,
        1,
        NULL,
        0
    );
}

void PL_Telemetry_ESP32::sendSnapshot(const float* values, uint64_t timestamp) {
    if(!_telemetryStarted) return;

    InternalSnapshot snap;
    for(size_t i=0;i<_numVars;i++) {
        snap.vars[i] = values[i];
    }
    snap.timestamp_us = timestamp;

    xQueueSend(_snapshotQueue, &snap, 0);
}
