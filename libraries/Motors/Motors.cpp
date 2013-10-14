#include "Motors.h"

Motor::Motor(String name, int pinA, int pinB, int pinEnable){
	_pinA = pinA;
	_pinB = pinB;
	_pinEnable = pinEnable;
	_pwm = 0;
	_name = name;

	pinMode(_pinA, OUTPUT);
	pinMode(_pinB, OUTPUT);
	pinMode(_pinEnable, OUTPUT);

	digitalWrite(_pinA, LOW);
	digitalWrite(_pinB, LOW);

}

void Motor::stop(){
	int _pwm = 0;
	digitalWrite(_pinA, LOW);
	digitalWrite(_pinB, LOW);
	analogWrite(_pinEnable, _pwm);
	plotSpeed(STOP);
}

void Motor::driveForwards(int pwm){
	int _pwm = pwm;
	#ifdef MOTORS_ON
		analogWrite(_pinEnable, _pwm);
		digitalWrite(_pinA, HIGH);
		digitalWrite(_pinB, LOW);
	#endif
	plotSpeed(FORWARDS);
}

void Motor::driveBackwards(int pwm){
	int _pwm = pwm;
	#ifdef MOTORS_ON
		analogWrite(_pinEnable, _pwm);
		digitalWrite(_pinA, LOW);
		digitalWrite(_pinB, HIGH);
	#endif
	plotSpeed(BACKWARDS);
}

void Motor::plotSpeed(int direction){
	#ifdef PLOT_PRINT_MOTORS_ON
		PLOT(_name + "Speed", _pwm);
		PLOT(_name + "Direction", direction);
	#endif
}