#include "IRSensor.h"

IRSensor::IRSensor(String name, int pinLED, int pinFront, int pinSide, int frontThresh, int sideThresh) : _frontFilter(IR_FILTER_SIZE), _sideFilter(IR_FILTER_SIZE) {
	_pinLED = pinLED;
	_pinFront = pinFront;
	_pinSide = pinSide;
	_filterSize = IR_FILTER_SIZE;
	_name = name;

	_frontThresh = frontThresh;
	_sideThresh = sideThresh;

	_frontOff = 0;
	_frontOn = 0;
	_frontDiff = 0;

	_sideOff = 0;
	_sideOn = 0;
	_sideDiff = 0;

	_frontDiffFilter = 0;
	_sideDiffFilter = 0;

	_frontDiffFilterLarge = 0;
	_sideDiffFilterLarge = 0;

	pinMode(_pinLED, OUTPUT);
	pinMode(_pinFront, INPUT);
	pinMode(_pinSide, INPUT);
}

void IRSensor::update(){
	_frontOff = analogRead(_pinFront);
	LED_READ_DELAY;
	_frontOff = analogRead(_pinFront);

	_sideOff = analogRead(_pinSide);
	LED_READ_DELAY;
	_sideOff = analogRead(_pinSide);

	digitalWrite(_pinLED, HIGH);

	_frontOn = analogRead(_pinFront);
	LED_READ_DELAY;
	_frontOn = analogRead(_pinFront);

	_sideOn = analogRead(_pinSide);
	LED_READ_DELAY;
	_sideOn = analogRead(_pinSide);

	digitalWrite(_pinLED, LOW);

	_frontDiff = _frontOn - _frontOff;
	_sideDiff = _sideOn - _sideOff;

	_frontDiffFilterLarge -= _frontDiffFilterLarge/_filterSize;
	_frontDiffFilterLarge += _frontDiff;
	_frontDiffFilter = _frontDiffFilterLarge/_filterSize;

	_sideDiffFilterLarge -= _sideDiffFilterLarge/_filterSize;
	_sideDiffFilterLarge += _sideDiff;
	_sideDiffFilter = _sideDiffFilterLarge/_filterSize;

	_frontFilter.update(_frontDiffFilter > _frontThresh);
	_sideFilter.update(_sideDiffFilter > _sideThresh);

  	#ifdef PLOT_PRINT_IR_ON_DETAIL
  		PLOT(_name + "_FRONT_OFF", _frontOff);
	    PLOT(_name + "_FRONT_ON", _frontOn);
		PLOT(_name + "_FRONT_DIFF", _frontDiff);
		PLOT(_name + "_FRONT_DIFF_FILTER", _frontDiffFilter);

	    PLOT(_name + "_SIDE_OFF", _sideOff);
	    PLOT(_name + "_SIDE_ON", _sideOn);
	    PLOT(_name + "_SIDE_DIFF", _sideDiff);
		PLOT(_name + "_SIDE_DIFF_FILTER", _sideDiffFilter);
    #endif

	#ifdef PLOT_PRINT_IR_ON
		PLOT(_name + "_FRONT_DIFF_FILTER", _frontDiffFilter);
		PLOT(_name + "_SIDE_DIFF_FILTER", _sideDiffFilter);
    #endif
}

int IRSensor::getFront(){
	return _frontDiffFilter;
}
int IRSensor::getSide(){
	return _sideDiffFilter;
}

bool IRSensor::frontOn(){
	return _frontFilter.on();
}
bool IRSensor::sideOn(){
	return _sideFilter.on();
}

int IRSensor::frontGetTimeSinceChange(){
	return _frontFilter.getTimeSinceChange();
}
int IRSensor::sideGetTimeSinceChange(){
	return _sideFilter.getTimeSinceChange();
}

void IRSensor::frontResetTimeSinceChange(){
	_frontFilter.resetTimeSinceChange();
}

void IRSensor::sideResetTimeSinceChange(){
	_frontFilter.resetTimeSinceChange();
}	
