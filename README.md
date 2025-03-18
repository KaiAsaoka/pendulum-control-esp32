# 2511 2D Inverted Pendulum ESP32 Firmware

This repo contains the firmware code for gantry control for a 2D inverted pendulum project as a part of a UBC Engineering Physics Capstone project.

---

## Project TODO List

- **Driver Dynamics**
  - [ ] Implement Move.h to map motor rotation to cartesian movement of the gantry
- **ESP32 Bluetooth Communication**
  - [ ] Implement and test BTSerial class to allow communication between motor driver and pendulum ESP32s.
- **Monitoring Suite**
  - [ ] Implement Python script to read serial output out of ESPs
- **Documentation**
  - [ ] Add comments to all files for transition to ENPH 479

---

## Additional Information

- **Hardware:**  
  ESP32-based board with hall-effect encoder sensors and motor driver.

- **Software:**  
  The project uses the Arduino framework with PlatformIO as the build system.

- **Next Steps:**  
  Work through tasks above sequentially and mark each task as completed as progress is made.