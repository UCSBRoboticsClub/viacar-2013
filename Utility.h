#ifndef _UTILITY_H
#define _UTILITY_H



inline int deadzone(int input, int zone)
{
    return (input > zone || input < -zone) ? input : 0;
}

inline float pos(float f)
{
	return (f > 0.0f) ? f : 0.0f;
}

inline float min(float a, float b)
{
	return (a < b) ? a : b;
}

#endif // _UTILITY_H