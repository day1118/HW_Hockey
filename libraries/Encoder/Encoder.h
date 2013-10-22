#include "Arduino.h"
#include "Config.h"
#include "../Filter/Filter.h"

class Encoder
{
public:
	Encoder(String name, int pinLED, int pinPhototransistor, int thresh);
	void update();
	int getValue();
	bool on();
	int getTimeSinceChange();
	void resetTimeSinceChange();

private:
	Filter _filter;

	int _pinLED;
	int _pinPhototransistor;
	
	int _filterSize;

	int _off;
	int _on;
	int _diff;

	int _diffFilter;

	long _diffFilterLarge;

	String _name;

	int _thresh;
};