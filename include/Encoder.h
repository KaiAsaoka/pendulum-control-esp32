#ifndef Encoder_H
#define Encoder_H

class Encoder {
public:
    Encoder(int miso, int clk, int cs, int mosi, int numBits);
    Encoder(int miso, int clk, int cs, int mosi); // Overloaded constructor with default numBits
    void begin();
    int readAngle();       // Returns current angle (0-360) and updates rotation count
    long getTotalAngle();   // Returns total angle as (rotationCount * 360 + current angle)
    float getTotalAngleFloat();   // Returns total angle as (rotationCount * 360 + current angle)
    void zero();

    
    // Static member variable shared across all instances
    static int firstReading;  // Declaration of static member

private:
    int miso;
    int clk;
    int cs;
    int mosi;
    int mask;           // Mask for valid bits (e.g., 0x3FFF for 14 bits)
    
    float prevAngle;    // Previous instantaneous angle
    int rotationCount;  // Count of full rotations (+/-)
    int zeroAngle;      // Zero angle
};

#endif // Encoder_H