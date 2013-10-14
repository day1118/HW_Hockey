#include "Arduino.h"
#include "Config.h"

class IRSensor
{
public:
	IRSensor(String name, int pinLED, int pinFront, int pinSide);
	void update();
	int getFront();
	int getSide();	

//private:
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
};