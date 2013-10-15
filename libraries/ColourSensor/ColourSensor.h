#include "Arduino.h"
#include "Config.h"

class ColourSensor
{
public:
	ColourSensor(String name, int pinLED1, int pinLED2, int pinPhototransistor);
	void update();
	int getColour1();
	int getColour2();	

//private:
	int _pinLED1;
	int _pinLED2;
	int _pinPhototransistor;
	int _filterSize;
	String _name;

	int _off;
	int _colour1On;
	int _colour2On;

	int _colour1Diff;
	int _colour2Diff;

	int _colour1DiffFilter;
	int _colour2DiffFilter;

	long _colour1DiffFilterLarge;
	long _colour2DiffFilterLarge;
};