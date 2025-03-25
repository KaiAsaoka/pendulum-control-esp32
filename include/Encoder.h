#ifndef Encoder_H
#define Encoder_H

class Encoder {
public:
    Encoder(int miso, int clk, int cs, int mosi);
    void begin();
    int readAngle();       // Returns current angle (0-360) and updates rotation count
    float getTotalAngle();   // Returns total angle as (rotationCount * 360 + current angle)
    
    // Static member variable shared across all instances
    static int firstReading;  // Declaration of static member

private:
    int miso;
    int clk;
    int cs;
    int mosi;
    
    float prevAngle;    // Previous instantaneous angle
    int rotationCount;  // Count of full rotations (+/-)
};

#endif // Encoder_H