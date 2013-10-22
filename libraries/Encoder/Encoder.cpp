#include "Encoder.h"

Encoder::Encoder(String name, int pinLED, int pinPhototransistor, int thresh) : _filter(ENCODER_FILTER_SIZE) {
	_pinLED = pinLED;
	_pinPhototransistor = pinPhototransistor;
	_filterSize = ENCODER_FILTER_SIZE;
	_name = name;

	_thresh = thresh;
	
	_off = 0;
	_on = 0;
	_diff = 0;

	_diffFilter = 0;
	_diffFilterLarge = 0;

	pinMode(_pinLED, OUTPUT);
	pinMode(_pinPhototransistor, INPUT);
}

void Encoder::update(){
	_off = analogRead(_pinPhototransistor);
	LED_READ_DELAY;
	_off = analogRead(_pinPhototransistor);

	digitalWrite(_pinLED, HIGH);

	_on = analogRead(_pinPhototransistor);
	LED_READ_DELAY;
	_on = analogRead(_pinPhototransistor);

	digitalWrite(_pinLED, LOW);

	_diff = _on - _off;

	_diffFilterLarge -= _diffFilterLarge/_filterSize;
	_diffFilterLarge += _diff;
	_diffFilter = _diffFilterLarge/_filterSize;

	_filter.update(_diffFilter > _thresh);

  	#ifdef PLOT_PRINT_ENCODER_ON_DETAIL
  		PLOT(_name + "_OFF", _off);
	    PLOT(_name + "_ON", _on);
		PLOT(_name + "_DIFF", _diff);
		PLOT(_name + "_FILTER", _diffFilter);
    #endif

	#ifdef PLOT_PRINT_ENCODER_ON
		PLOT(_name + "_DIFF_FILTER", _diffFilter);
    #endif
}

int Encoder::getValue(){
	return _diffFilter;
}

bool Encoder::on(){
	return _filter.on();
}

int Encoder::getTimeSinceChange(){
	return _filter.getTimeSinceChange();
}

void Encoder::resetTimeSinceChange(){
	_filter.resetTimeSinceChange();
}
