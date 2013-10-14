#include "Filter.h"

Filter::Filter(int thresh){
	_counter = 0;
	_active = LOW;
	_thresh = thresh;
	_changeTime = millis();
}

void Filter::update(bool newState){
	if(_active == newState)
	{
		if(_counter < _thresh)
			_counter++;
	}
	else
	{
		if(_counter > 0)
			_counter++;
		else
		{
			_active = newState;
			_changeTime = millis();
		}
	}
}

unsigned long Filter::getTimeSinceChange(){
	return millis() - _changeTime;
}

bool Filter::on(){
	return _active;
}