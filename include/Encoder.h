#ifndef Encoder_H
#define Encoder_H

class Encoder {
public:
    Encoder(int miso, int clk, int cs, int mosi);
    void begin();
    float readAngle();       // Returns current angle (0-360) and updates rotation count
    float getTotalAngle();   // Returns total angle as (rotationCount * 360 + current angle)

private:
    int miso;
    int clk;
    int cs;
    int mosi;
    
    bool firstReading;  // True on the first call to readAngle()
    float prevAngle;    // Previous instantaneous angle
    int rotationCount;  // Count of full rotations (+/-)
};

#endif // Encoder_H