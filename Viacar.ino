#include <WProgram.h>
#include "./Motor.h"
#include "./RadioTerminal.h"
#include "./Servo.h"
#include "./Utility.h"
#include "./CircularBuffer.h"
#include <cstdio>
#include <cmath>



float throttle, turn;
Motor motor(5);
Servo steering(3);
int cyclecount = 0;
int msstart;

float kp, ki, kd;
float c1, c2, hsq, d;
float error, ierror, derror;
CircularBuffer<float, 2> prevError;
const float period = 0.02f;



float getError()
{
	float v1 = analogRead(A2)/4096.0f;
	float v2 = analogRead(A3)/4096.0f;
	
	float x1 = sqrt(pos(c1/(v1*v1) - hsq));
	float x2 = sqrt(pos(c2/(v2*v2) - hsq));
	
	float aa = abs(d + x1 - x2);
	float ab = abs(d + x1 + x2);
	float ba = abs(d - x1 - x2);
	float bb = abs(d - x1 + x2);
	
	float m = min(min(aa, ab), min(ba, bb));
	
	if (m == aa) return (x1 + x2) / 2.0f;
	else if (m == ab) (x1 - x2) / 2.0f;
	else if (m == ba) (x2 - x1) / 2.0f;
	else return (x1 + x2) / -2.0f;
}



void setup()
{
	Serial.begin(115200);
	
    RadioTerminal::initialize(10, 14, 15);
    RadioTerminal::reset();
	
	analogReadRes(12);
	analogReference(INTERNAL);
	analogReadAveraging(16);
	
	c1 = 1.0f;
	c2 = 1.0f;
	hsq = 0.02f;
	d = 0.1f;
	
	msstart = millis();
}



void loop()
{
	// Update the main loop at 50 Hz
	while (millis() - msstart < cyclecount * (int)(period * 1000));
	cyclecount++;
	
	// Get current error and calculate derivative and integral
	error = getError();
	ierror += (error + prevError[0]) / 2.0f * period;
	derror = (3.0f*error - 4.0f*prevError[0] + prevError[1]) / (2.0f*period);
	prevError.push(error);

	// Use manual steering if a controller message is present
	if (RadioTerminal::rx_controller != 0)
	{
		turn = 0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>0)&0xff), 8); // Convert to +/-1.0f range
		throttle = -0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>8)&0xff), 8);
	}
	else
	{
		// Use PID control if no controller is detected
		throttle = 0.5f;
		turn = kp*error + ki*ierror + kd*derror;
	}
	
	steering = turn * 60.0f;
	motor = throttle;
}
