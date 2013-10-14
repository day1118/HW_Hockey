#include "Arduino.h"

class Filter{
public:
	Filter(int thresh);
	void update(bool newState);
	bool on();
	unsigned long getTimeSinceChange();

private:
	int _counter;
	bool _active;
	int _thresh;
	unsigned long _changeTime;
};