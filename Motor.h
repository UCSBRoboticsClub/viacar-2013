#ifndef _MOTOR_H
#define _MOTOR_H



class Motor
{
public:
    Motor(int speedPin);
    void write(float value);
    Motor& operator=(float value);
    
private:
    int speedPin;
};

#endif // _MOTOR_H