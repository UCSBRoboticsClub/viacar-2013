#include <WProgram.h>
#include "./Motor.h"
#include "./RadioTerminal.h"
#include "./Servo.h"



float throttle, turn;
Motor motor(5);
Servo steering(3);



int deadzone(int input, int zone)
{
    return (input > zone || input < -zone) ? input : 0;
}



void setup()
{
    RadioTerminal::initialize(10, 14, 15);
    RadioTerminal::reset();
    
    Serial.begin(115200);
	
	pinMode(13, OUTPUT);
    digitalWrite(13, 1);
}



void loop()
{
    throttle = 0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>0)&0xff), 8); // Convert to +/-1.0f range
    turn = -0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>8)&0xff), 8);
    
    motor = throttle;
    steering = turn * 60.0f;
    
    Serial.println(RadioTerminal::rx_controller);
}
