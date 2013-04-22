#include <WProgram.h>
#include "./Motor.h"
#include "./RadioTerminal.h"
#include "./Servo.h"
#include "./Utility.h"
#include "./CircularBuffer.h"
#include "./PITimer.h"
#include <cstdio>
#include <cmath>



float throttle, turn;
Motor motor(5);
Servo steering(3);
int cyclecount = 0;
int msstart;

float kp, ki, kd;
float v1, v2, x1, x2;
float hsq1, hsq2, d;
float error, ierror, derror;
CircularBuffer<float, 2> prevError;
const float period = 0.02f;



class WatchHandler : public CmdHandler
{
public:
	WatchHandler(float* watchAddr);
    virtual void sendChar(char c) { PITimer1.stop(); RadioTerminal::terminateCmd(); }
    static float* watch;
	
    static void refresh();
};

float* WatchHandler::watch;



CmdHandler* watch(const char* input)
{
    // Read command parameters
    if (!strncmp(input, "w v1", 8))
    {
        return new WatchHandler(&v1);
    }
	else if (!strncmp(input, "w v2", 8))
    {
        return new WatchHandler(&v2);
    }
	else if (!strncmp(input, "w x1", 8))
    {
        return new WatchHandler(&x1);
    }
	else if (!strncmp(input, "w x2", 8))
    {
        return new WatchHandler(&x2);
    }
	else if (!strncmp(input, "w error", 8))
    {
        return new WatchHandler(&error);
    }
	else if (!strncmp(input, "w ierror", 8))
    {
        return new WatchHandler(&ierror);
    }
	else if (!strncmp(input, "w derror", 8))
    {
        return new WatchHandler(&derror);
    }
    else
    {
        RadioTerminal::write("error reading input parameters");
        return NULL;
    }
}



WatchHandler::WatchHandler(float* watchAddr)
{
    watch = watchAddr;
    PITimer1.period(0.2f);
	PITimer1.start(&WatchHandler::refresh);
}



void WatchHandler::refresh()
{
    char output[256];

    sprintf(output, "\r         \r\r\r%4.4f", *watch);
    RadioTerminal::write(output);
}



CmdHandler* setkp(const char* input)
{
    char output[256];
	
    sscanf(input, "kp %f", &kp);
    sprintf(output, "kp = %f", kp);
    RadioTerminal::write(output);
    
    return NULL;
}



CmdHandler* setki(const char* input)
{
    char output[256];
    
    sscanf(input, "ki %f", &ki);
    sprintf(output, "ki = %f", ki);
    RadioTerminal::write(output);
    
    return NULL;
}



CmdHandler* setkd(const char* input)
{
    char output[256];
    
    sscanf(input, "kd %f", &kd);
    sprintf(output, "kd = %f", kd);
    RadioTerminal::write(output);
    
    return NULL;
}



CmdHandler* sethsq1(const char* input)
{
    char output[256];
    
    sscanf(input, "hsq1 %f", &hsq1);
    sprintf(output, "hsq1 = %f", hsq1);
    RadioTerminal::write(output);
    
    return NULL;
}



CmdHandler* sethsq2(const char* input)
{
    char output[256];
    
    sscanf(input, "hsq2 %f", &hsq2);
    sprintf(output, "hsq2 = %f", hsq2);
    RadioTerminal::write(output);
    
    return NULL;
}



CmdHandler* setd(const char* input)
{
    char output[256];
    
    sscanf(input, "d %f", &d);
    sprintf(output, "d = %f", d);
    RadioTerminal::write(output);
    
    return NULL;
}



float getError(float predicted)
{
	v1 = analogRead(A2)/4096.0f;
	v2 = analogRead(A3)/4096.0f;
	
	x1 = sqrt(pos(hsq1/v1 - hsq1));
	x2 = sqrt(pos(hsq2/v2 - hsq2));
	
	float daa = abs(d + x1 - x2);
	float dab = abs(d + x1 + x2);
	float dba = abs(d - x1 - x2);
	float dbb = abs(d - x1 + x2);
	
	float aa = (x1 + x2) / 2.0f;
	float ab = (x1 - x2) / 2.0f;
	float ba = (x2 - x1) / 2.0f;
	float bb = (x1 + x2) / -2.0f;
	
	float eaa = abs(abs(aa) - predicted);
	float eab = abs(abs(ab) - predicted);
	float eba = abs(abs(ba) - predicted);
	float ebb = abs(abs(bb) - predicted);
	
	float taa = daa + eaa;
	float tab = dab + eab;
	float tba = dba + eba;
	float tbb = dbb + ebb;
	
	float m = taa;
	float res = aa;
	
	if (m > tab)
	{
		m = tab;
		res = ab;
	}
	if (m > tba)
	{
		m = tba;
		res = ba;
	}
	if (m > tbb)
	{
		m = tbb;
		res = bb;
	}
	
	return res;
}



void setup()
{
    RadioTerminal::initialize(10, 14, 15);
    RadioTerminal::reset();
	
	RadioTerminal::addCommand("kp", &setkp);
	RadioTerminal::addCommand("ki", &setki);
	RadioTerminal::addCommand("kd", &setkd);
	RadioTerminal::addCommand("hsq1", &sethsq1);
	RadioTerminal::addCommand("hsq2", &sethsq2);
	RadioTerminal::addCommand("d", &setd);
	RadioTerminal::addCommand("w", &watch);
	
	analogReadRes(12);
	analogReference(INTERNAL);
	analogReadAveraging(16);
	
	hsq1 = 4.0f;
	hsq2 = 4.0f;
	d = 0.1f;
	
	kp = 1.0f;
	ki = 0.5f;
	kd = 0.01f;
	
	msstart = millis();
}



void loop()
{
	// Update the main loop at 50 Hz
	while (millis() - msstart < cyclecount * (int)(period * 1000));
	cyclecount++;
	
	// Get current error and calculate derivative and integral
	error = getError(error + derror*period);
	ierror += (error + prevError[0]) / 2.0f * period;
	derror = (3.0f*error - 4.0f*prevError[0] + prevError[1]) / (2.0f*period);
	prevError.push(error);

	// Use manual steering if a controller message is present
	if (RadioTerminal::rx_controller != 0)
	{
		turn = 0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>16)&0xff), 8); // Convert to +/-1.0f range
		throttle = -0.0078125f * deadzone((int8_t)((RadioTerminal::rx_controller>>8)&0xff), 8);
	}
	else
	{
		// Use PID control if no controller is detected
		throttle = 0.0f;//0.5f;
		turn = 0.0f;//kp*error + ki*ierror + kd*derror;
	}
	
	steering = turn * 50.0f;
	motor = throttle;
}
