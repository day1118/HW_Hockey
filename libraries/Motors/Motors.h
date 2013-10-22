#include "Arduino.h"
#include "Config.h"

class Motor
{

public:
	Motor(String name, int pinA, int pinB, int pinEnable);
	Motor(String name);
	void stop();
	void driveForwards(int pwm);
	void driveBackwards(int pwm);

private:
	void plotSpeed(int direction);
	int limit(int pwm);
	int _pinA;
	int _pinB;
	int _pinEnable;
	int _pwm;
	bool _enabled;
	String _name;
};