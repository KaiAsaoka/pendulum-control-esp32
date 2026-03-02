    #include "PL_Telemetry_ESP32.h"
    #include <iostream>

    void PL_Telemetry_ESP32::beginSerial() {
        Serial.begin(2000000);
        while (!Serial) delay(10);
        _serialStarted = true;
        Serial.println("Serial Telemetry Initialized");
    }

    void PL_Telemetry_ESP32::sendPacket(uint8_t* buffer, size_t size) {
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
        
        // Serial.write(buffer, offset);

        offset++;
        sendPacket(buffer, offset); 
    }

    void PL_Telemetry_ESP32::sendPID() {
        uint8_t buffer[128 + 1];
        size_t offset = 0;
        size_t offsetIncrement = sizeof(int);

        buffer[offset++] = 0xCD;
        buffer[offset++] = 0xAC;
        buffer[offset++] = 3 + 20 * sizeof(int);

        for (pidParams* paramSet : _pidParams) {
            memcpy(buffer + offset, &paramSet->p, sizeof(int));
            offset += sizeof(int);
            memcpy(buffer + offset, &paramSet->i, sizeof(int));
            offset += sizeof(int);
            memcpy(buffer + offset, &paramSet->d, sizeof(int));
            offset += sizeof(int);
            memcpy(buffer + offset, &paramSet->lpf, sizeof(int));
            offset += sizeof(int);
            memcpy(buffer + offset, &paramSet->iCutoff, sizeof(int));
            offset += sizeof(int);
        }

        // Serial.write(buffer, offset)
        offset++;
        sendPacket(buffer, offset);
    }

    void PL_Telemetry_ESP32::readGainVals() {
        size_t expectedBytes = 20 * sizeof(int);
        uint8_t buf[expectedBytes + 1];
        size_t bytesRead = 0;

        while (bytesRead < expectedBytes) {
            if (Serial.available()) {
                buf[bytesRead++] = Serial.read();
            }
        }

        size_t offset = 0;

        for (pidParams* paramSet : _pidParams) {
            memcpy(&paramSet->p, buf+offset, sizeof(int));
            offset += sizeof(int);
            memcpy(&paramSet->i, buf+offset, sizeof(int));
            offset += sizeof(int);
            memcpy(&paramSet->d, buf+offset, sizeof(int));
            offset += sizeof(int);
            memcpy(&paramSet->lpf, buf+offset, sizeof(int));
            offset += sizeof(int);
            memcpy(&paramSet->iCutoff, buf+offset, sizeof(int));
            offset += sizeof(int);
        }

        _pidReceive.store(true, std::memory_order_release);
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
            _testingPaused.store(false, std::memory_order_release);
            Serial.println("START received!");
        }
        else if(strcmp(buf,"STOP") == 0) {
            _testingPaused.store(true, std::memory_order_release);
            Serial.println("STOP recieved!");
        }
        else if (strcmp(buf,"SENDPID") == 0) {
            _pidSent = true;
            Serial.println("PID sending!");
            sendPID();
        }
        else if (strcmp(buf,"PIDRECV") == 0) {
            Serial.println("PID received!");
            vTaskDelay(pdMS_TO_TICKS(10));
            readGainVals();
        }
        else if (strcmp(buf,"END") == 0) {
            _pidSent = false;
            _telemetryStarted = false;
            _metadataRequested = false;
        }
    }

    void PL_Telemetry_ESP32::telemetryTask() {
        // Use internal snapshot array
        InternalSnapshot batch[_BATCH_SIZE];

        // FIX: Pre-allocate the maximum buffer size once. No more heap fragmentation.
        const size_t MAX_PACKET_SIZE = sizeof(TelemetryPacketHeader) + _BATCH_SIZE * (sizeof(float) * _numVars + sizeof(uint64_t)) + 3;
        uint8_t static_buffer[MAX_PACKET_SIZE];

        beginSerial();

        for (;;) {
            checkCommands();

            if (!_telemetryStarted || _testingPaused) {
                xQueueReset(_snapshotQueue);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
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

                // Calculate the actual size of this specific packet
                size_t packetSize = sizeof(TelemetryPacketHeader) + count * (sizeof(float) * _numVars + sizeof(uint64_t)) + 3;

                // FIX: Use the static buffer instead of dynamically allocating a new one
                TelemetryPacketHeader* header = (TelemetryPacketHeader*)static_buffer;
                header->sync = 0xAA55;
                header->seq = _packetSeq++;
                header->num_snapshots = count;
                header->num_vars = _numVars;

                uint8_t* ptr = static_buffer + sizeof(TelemetryPacketHeader);
                for (uint8_t i = 0; i < count; i++) {
                    memcpy(ptr, batch[i].vars, _numVars * sizeof(float));
                    ptr += _numVars * sizeof(float);
                    memcpy(ptr, &batch[i].timestamp_us, sizeof(uint64_t));
                    ptr += sizeof(uint64_t);
                }

                // CRC placeholder
                uint16_t* crcPtr = (uint16_t*)(static_buffer + packetSize - 3);
                *crcPtr = 0xFFFF;   

                // Send packet
                sendPacket(static_buffer, packetSize); 

                // FIX: Removed 'delete[] buffer' because we are using the static array now

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

    bool PL_Telemetry_ESP32::updateGainVals() {
        return _pidReceive.exchange(false, std::memory_order_acq_rel);
    }

    bool PL_Telemetry_ESP32::pauseTesting() {
        return _testingPaused;
    }