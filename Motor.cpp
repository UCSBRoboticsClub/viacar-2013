#include "./Motor.h"
#include <WProgram.h>
#include "./Hardware.h"



Motor::Motor(int speedPin)
{
    this->speedPin = speedPin;
    
    pinMode(speedPin, OUTPUT);
    
    // Set PWM frequency to 40kHz
    analogWriteFrequency(speedPin, 40000);
    pwmWrite(speedPin, 0.0f);
}



void Motor::write(float value)
{
    // Set speed
    pwmWrite(speedPin, (value > 0.0f) ? value : 0.0f);
}



Motor& Motor::operator=(float value)
{
    write(value);
    return *this;
}
