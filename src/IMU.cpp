#include <Arduino.h>
#include <SPI.h>

#ifndef SPI_MODE_3
#define SPI_MODE_3 3
#endif

// --- SPI Pins ---
// (Make sure these match your actual ESP32 wiring)
#define IMU_MISO 27
#define IMU_MOSI 0
#define IMU_CLK  14
#define IMU_CS   5 

// --- IMU Registers & Constants ---
#define REG_GYRO_XOUT_H 0x43
#define READ_BIT 0x80           // ADDED: Required for SPI reading
#define GYRO_SENSITIVITY 131.0f // LSB/(°/s) for ±250 dps range
#define IMU_LOOP_US 100000      // 100 ms

// Helper function to write to registers
void writeRegister(uint8_t reg, uint8_t data) {
    digitalWrite(IMU_CS, LOW);
    SPI.transfer(reg); // Write bit is 0, so no OR needed
    SPI.transfer(data);
    digitalWrite(IMU_CS, HIGH);
}

void setupIMU() {
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    // MPU6500 supports up to 20MHz for data, 
    // but 1MHz is safer for initial configuration.
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE_3));
    
    // Wake up: Register 0x6B (PWR_MGMT_1), Data 0x00
    writeRegister(0x6B, 0x00); 
    
    SPI.endTransaction();
}

// ADDED: Standard Arduino setup function
void setup() {
    Serial.begin(115200);
    
    // Initialize the SPI bus
    SPI.begin(IMU_CLK, IMU_MISO, IMU_MOSI); 
    
    setupIMU();
    Serial.println("IMU Initialized! Starting reads...");
}

void loop() {
    static uint32_t last_run = micros();

    if (micros() - last_run >= IMU_LOOP_US) {
        last_run = micros();

        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE_3));
        
        // Read 6 bytes starting from GYRO_XOUT_H
        // (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
        digitalWrite(IMU_CS, LOW);
        SPI.transfer(REG_GYRO_XOUT_H | READ_BIT);
        
        int16_t rawX = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
        int16_t rawY = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
        int16_t rawZ = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
        
        digitalWrite(IMU_CS, HIGH);
        SPI.endTransaction();

        // Convert to degrees per second
        float gyroX = rawX / GYRO_SENSITIVITY;
        float gyroY = rawY / GYRO_SENSITIVITY;
        float gyroZ = rawZ / GYRO_SENSITIVITY;

        Serial.printf("X: %6.2f | Y: %6.2f | Z: %6.2f °/s\n", gyroX, gyroY, gyroZ);
    }
}