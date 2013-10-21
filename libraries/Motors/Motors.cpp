#include "Motors.h"

Motor::Motor(String name, int pinA, int pinB, int pinEnable){
	_pinA = pinA;
	_pinB = pinB;
	_pinEnable = pinEnable;
	_pwm = 0;
	_name = name;
	_enabled = true;

	pinMode(_pinA, OUTPUT);
	pinMode(_pinB, OUTPUT);
	pinMode(_pinEnable, OUTPUT);

	digitalWrite(_pinA, LOW);
	digitalWrite(_pinB, LOW);
	analogWrite(_pinEnable, _pwm);
}

Motor::Motor(String name){
_name = name;
_enabled = false;
}

void Motor::stop(){
	int _pwm = 0;
	if(_enabled)
	{
		digitalWrite(_pinA, LOW);
		digitalWrite(_pinB, LOW);
		analogWrite(_pinEnable, _pwm);
	}
	plotSpeed(DIRECTION_STOP);
}

void Motor::driveForwards(int pwm){
	int _pwm = pwm;
	#ifdef MOTORS_ON
		if(_enabled)
		{
			analogWrite(_pinEnable, _pwm);
			digitalWrite(_pinA, HIGH);
			digitalWrite(_pinB, LOW);
		}
	#endif
	plotSpeed(DIRECTION_FORWARDS);
}

void Motor::driveBackwards(int pwm){
	int _pwm = pwm;
	#ifdef MOTORS_ON
		if(_enabled)
		{
			analogWrite(_pinEnable, _pwm);
			digitalWrite(_pinA, LOW);
			digitalWrite(_pinB, HIGH);
		}
	#endif
	plotSpeed(DIRECTION_BACKWARDS);
}

void Motor::plotSpeed(int direction){
	#ifdef PLOT_PRINT_MOTORS_ON
		PLOT(_name + "_Speed", _pwm);
		PLOT(_name + "_Direction", direction);
	#endif
}