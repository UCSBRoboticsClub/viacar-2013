#include <WProgram.h>
#include "./Motor.h"
#include "./RadioTerminal.h"
#include "./Servo.h"
#include <cstdio>



float throttle, turn;
Motor motor(5);
Servo steering(3);
int mscount = 0;
int msstart;



CmdHandler* loadavg(const char* string)
{
	char buffer[256];
	int mselapsed = millis() - msstart;
	
	sprintf(buffer, "%.2f%%", ((float)(mselapsed - mscount)) / ((float)mselapsed) * 100.0f);
	
	RadioTerminal::write(buffer);
	
	return NULL;
}



int deadzone(int input, int zone)
{
    return (input > zone || input < -zone) ? input : 0;
}



void setup()
{
	Serial.begin(115200);
	
    RadioTerminal::initialize(10, 14, 15);
    RadioTerminal::reset();
	
	RadioTerminal::addCommand("loadavg", &loadavg);
	
	msstart = millis();
}



void loop()
{
	throttle = 0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>0)&0xff), 8); // Convert to +/-1.0f range
    turn = -0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>8)&0xff), 8);
    
    motor = throttle;
    steering = turn * 60.0f;
	
	delayMicroseconds(1000);
	mscount++;
}
