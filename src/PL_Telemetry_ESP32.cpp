    #include "PL_Telemetry_ESP32.h"
    #include <iostream>

    void PL_Telemetry_ESP32::beginSerial() {
        Serial.begin(230400);
        while (!Serial) delay(10);
        _serialStarted = true;
        Serial.println("Serial Telemetry Initialized");
    }

    bool PL_Telemetry_ESP32::pauseTesting() {
        return(!_telemetryStarted);
    }

    void PL_Telemetry_ESP32::sendPacket(uint8_t* buffer, size_t size) {
        buffer[size - 1] = 0x0A;
        Serial.write(buffer, size);
    }

    void PL_Telemetry_ESP32::sendMetadata() {
        uint8_t buffer[512];
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
        
        offset++;
        sendPacket(buffer, offset); 
    }

    void PL_Telemetry_ESP32::sendPID() {
        uint8_t buffer[128 + 1];
        size_t offset = 0;

        buffer[offset++] = 0xCD;
        buffer[offset++] = 0xAC;
        
        for (int i = 0; i < 20; i++) {
            float val = *(_pidGainVals[i]);
            memcpy(buffer + offset, &val, sizeof(float));
            offset += sizeof(float);
        }

        offset++;
        vTaskDelay(pdMS_TO_TICKS(10));
        sendPacket(buffer, offset);
    }

    void PL_Telemetry_ESP32::readGainVals() {
        size_t expectedBytes = 20 * sizeof(float);
        uint8_t buf[expectedBytes];
        size_t bytesRead = 0;

        while (Serial.available() > 0) {
            buf[bytesRead++] = Serial.read();
        }

        for (int i = 0; i < 20; i++) {
            float val;
            memcpy(&val, buf + i * sizeof(float), sizeof(float));
            *(_pidGainVals[i]) = val;
        }
        _pidReceive = false;
    }

    void PL_Telemetry_ESP32::checkCommands() {
        char buf[1024];
        uint8_t len = 0;
        // int packetSize = Serial.available();
        while(Serial.available() > 0) {
            buf[len++] = Serial.read();
        }

        buf[len] = '\0'; //Convert to str

        // if(packetSize) {
        //     int len = Serial.readBytesUntil('\n', buf, sizeof(buf)-1);
        // }

        if(strcmp(buf,"METADATA") == 0) {
            _metadataRequested = true;
            Serial.println("METADATA recieved!");
            vTaskDelay(pdMS_TO_TICKS(10));
            sendMetadata();
        }
        else if(strcmp(buf,"START") == 0) {
            _telemetryStarted = true;
            _lastPulseTime = millis();
            Serial.println("START received!");
        }
        else if(strcmp(buf,"STOP") == 0) {
            _telemetryStarted = false;
            _lastPulseTime = millis();
            Serial.println("STOP recieved!");
        }
        else if(strcmp(buf,"PULSE") == 0) {
            _lastPulseTime = millis();
        }
        else if (strcmp(buf,"SENDPID") == 0) {
            _lastPulseTime = millis();
            _pidSent = true;
            Serial.println("PID sending!");
            sendPID();
        }
        else if (strcmp(buf,"PIDRECV") == 0) {
            _pidReceive = true;
            _lastPulseTime = millis();
            Serial.println("PID received!");
        }
    }

    void PL_Telemetry_ESP32::telemetryTask() {
        // Use internal snapshot array
        InternalSnapshot batch[_BATCH_SIZE];
        beginSerial();

        for (;;) {
            checkCommands();

            if (!_telemetryStarted) {
                xQueueReset(_snapshotQueue);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (!_telemetryStarted && _pidReceive) {
                Serial.println("PID receiving...");
                if (Serial.available() >= 20 * sizeof(float)) {
                    readGainVals();
                    Serial.println("PID values updated!");
                    vTaskDelay(pdMS_TO_TICKS(10));
                    sendPID();
                }
            }
            if (_telemetryStarted && _metadataRequested && _pidSent) {
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
                size_t packetSize = sizeof(TelemetryPacketHeader) + count * (sizeof(float) * _numVars + sizeof(uint64_t)) + 3;
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
                uint16_t* crcPtr = (uint16_t*)(buffer + packetSize - 3);
                *crcPtr = 0xFFFF;   

                // Send packet
                sendPacket(buffer, packetSize); 

                delete[] buffer;

                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    void PL_Telemetry_ESP32::begin() {
        _snapshotQueue = xQueueCreate(200,sizeof(InternalSnapshot));
        xTaskCreatePinnedToCore(
            [](void* arg) {
                ((PL_Telemetry_ESP32*)arg)->telemetryTask();
            },
            "TelemetryTask",
            1024,
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