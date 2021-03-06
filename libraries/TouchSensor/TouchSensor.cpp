#include "TouchSensor.h"

TouchSensor::TouchSensor(String name, int pinTouch) : _filter(TOUCH_SENSOR_FILTER_SIZE) {
	_pinTouch = pinTouch;
	_name = name;

	_touchOn = LOW;
	_timeOfChange = millis();

	pinMode(pinTouch, INPUT);
}

void TouchSensor::update(){
	_touchOn = !digitalRead(_pinTouch);
	_filter.update(_touchOn);

	#ifdef PLOT_PRINT_TOUCH_ON_DETAIL
  		PLOT(_name + "_ON", _touchOn);
	    PLOT(_name + "_ON_FILTER", _filter.on());
		PLOT(_name + "_TIME", _filter.getTimeSinceChange());
    #endif

	#ifdef PLOT_PRINT_TOUCH_ON
		PLOT(_name + "_ON_FILTER", _filter.on());
    #endif
}

int TouchSensor::on(){
	return _filter.on();
}

int TouchSensor::getTimeSinceChange(){
	return _filter.getTimeSinceChange();
}
