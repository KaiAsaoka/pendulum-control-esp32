// #include <Arduino.h>
// #include <SPI.h>
// #include "SparkFun_BNO080_Arduino_Library.h"
// // SPI Bus Pins
// #define IMU_MISO 27
// #define IMU_MOSI 0
// #define IMU_CLK  14

// // BNO085 Specific Pins
// #define IMU_CS   5
// #define IMU_WAK  4   // Wake pin
// #define IMU_INT  16  // Interrupt pin
// #define IMU_RST  17  // Reset pin

// BNO080 myIMU;

// void setupIMU() {
//     Serial.begin(115200); // might start in other file
//     Serial.println("Starting BNO085 over SPI...");

//     // Initialize SPI bus
//     SPI.begin(IMU_CLK, IMU_MISO, IMU_MOSI); // might start in other file

//     // Initialize BNO085
//     if (myIMU.beginSPI(IMU_CS, IMU_WAK, IMU_INT, IMU_RST) == false) {
//         Serial.println("BNO085 not detected. Check wiring!");
//         while (1); // Halt on failure
//     }

//     // Request Rotation Vector at 400 Hz (2500 us)
//     myIMU.enableRotationVector(2500); 
    
//     Serial.println("BNO085 Initialized! Waiting for first packet...");
// }

// void loop() {
//     // Check for new data packet via INT pin
//     if (myIMU.dataAvailable() == true) {
        
//         // Convert radians to degrees
//         float roll  = (myIMU.getRoll()) * 180.0 / PI;   
//         float pitch = (myIMU.getPitch()) * 180.0 / PI;   
        
//         // Optional: Get angular velocity (deg/s)
//         float gyroX = myIMU.getGyroX() * 180.0 / PI;
//         float gyroY = myIMU.getGyroY() * 180.0 / PI;
//         float gyroZ = myIMU.getGyroZ() * 180.0 / PI;

//         Serial.printf("Orientation -> Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f deg\n", roll, pitch, yaw);
//         // Serial.printf("Rotation Speed -> X: %6.2f | Y: %6.2f | Z: %6.2f deg/s\n", gyroX, gyroY, gyroZ);
//     }
// }