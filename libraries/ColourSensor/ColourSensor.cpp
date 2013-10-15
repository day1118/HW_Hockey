#include "ColourSensor.h"

ColourSensor::ColourSensor(String name, int pinLED1, int pinLED2, int pinPhototransistor){
	_pinLED1 = pinLED1;
	_pinLED2 = pinLED2;
	_pinPhototransistor = pinPhototransistor;
	_filterSize = COLOUR_FILTER_SIZE;
	_name = name;

	_off = 0;
	_colour1On = 0;
	_colour2On = 0;

	_colour1Diff = 0;
	_colour2Diff = 0;

	_colour1DiffFilter = 0;
	_colour2DiffFilter = 0;

	_colour1DiffFilterLarge = 0;
	_colour2DiffFilterLarge = 0;

	pinMode(_pinLED1, OUTPUT);
	pinMode(_pinLED2, OUTPUT);
	pinMode(_pinPhototransistor, INPUT);
}

void ColourSensor::update(){
	_off = analogRead(_pinPhototransistor);

	digitalWrite(_pinLED1, HIGH);
	LED_READ_DELAY;
	_colour1On = analogRead(_pinPhototransistor);
	digitalWrite(_pinLED1, LOW);

	digitalWrite(_pinLED2, HIGH);
	LED_READ_DELAY;
	_colour2On = analogRead(_pinPhototransistor);
	digitalWrite(_pinLED2, LOW);

	_colour1Diff = _colour1On - _off;
	_colour2Diff = _colour2On - _off;

	_colour1DiffFilterLarge -= _colour1DiffFilter;
	_colour1DiffFilterLarge += _colour1Diff;
	_colour1DiffFilter = _colour1DiffFilterLarge/_filterSize;

	_colour2DiffFilterLarge -= _colour2DiffFilter;
	_colour2DiffFilterLarge += _colour2Diff;
	_colour2DiffFilter = _colour2DiffFilterLarge/_filterSize;

  	#ifdef PLOT_PRINT_COLOUR_ON_DETAIL
  		PLOT(_name + "_OFF", _off);

	    PLOT(_name + "_COLOUR1_ON", _colour1On);
		PLOT(_name + "_COLOUR1_DIFF", _colour1Diff);
		PLOT(_name + "_COLOUR1_DIFF_FILTER", _colour1DiffFilter);

	    PLOT(_name + "_COLOUR2_ON", _colour2On);
	    PLOT(_name + "_COLOUR2_DIFF", _colour2Diff);
		PLOT(_name + "_COLOUR2_DIFF_FILTER", _colour2DiffFilter);
    #endif

	#ifdef PLOT_PRINT_COLOUR_ON
		PLOT(_name + "_COLOUR1_DIFF_FILTER", _colour1DiffFilter);
		PLOT(_name + "_COLOUR2_DIFF_FILTER", _colour2DiffFilter);
    #endif
}

int ColourSensor::getColour1(){
	return _colour1DiffFilter;
}
int ColourSensor::getColour2(){
	return _colour2DiffFilter;
}
