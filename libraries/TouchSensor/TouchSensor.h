#include "Arduino.h"
#include "Config.h"
#include <Filter.h>

class TouchSensor
{
public:
	TouchSensor(String name, int pinTouch);
	void update();
	int on();
	int getTimeSinceChange();

private:
	//Filter _filter;

	int _pinTouch;
	String _name;

	bool _touchOn;
	long _timeOfChange;
};