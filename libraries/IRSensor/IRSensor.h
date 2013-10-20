#include "Arduino.h"
#include "Config.h"
#include "../Filter/Filter.h"

class IRSensor
{
public:
	IRSensor(String name, int pinLED, int pinFront, int pinSide, int frontThresh, int sideThresh);
	void update();
	int getFront();
	int getSide();	
	bool frontOn();
	bool sideOn();
	int frontGetTimeSinceChange();
	int sideGetTimeSinceChange();
	void frontResetTimeSinceChange();
	void sideResetTimeSinceChange();

private:
	Filter _frontFilter;
	Filter _sideFilter;

	int _pinLED;
	int _pinFront;
	int _pinSide;

	int _filterSize;

	int _frontOff;
	int _frontOn;
	int _frontDiff;

	int _sideOff;
	int _sideOn;
	int _sideDiff;

	int _frontDiffFilter;
	int _sideDiffFilter;

	long _frontDiffFilterLarge;
	long _sideDiffFilterLarge;

	String _name;

	int _frontThresh;
	int _sideThresh;
};