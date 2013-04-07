#ifndef SERVO_H
#define SERVO_H



class Servo
{
public:
    Servo(int pin, bool start = true);
    bool calibrate(int plus45, int minus45, float upperLimit, float lowerLimit);
    void write(float degrees);
    float read();
    void disable();
    void enable();
    void operator=(float degrees);
    operator float();
    
    
    
protected:
    int pwmPin;
    int pulseWidth;
    bool enabled;
    
    int center;
    float usPerDegree;
    float upperLimit;
    float lowerLimit;
};

#endif // SERVO_H