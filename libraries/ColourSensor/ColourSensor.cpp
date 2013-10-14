#include "ir.h"

IRDuo::IRDuo(int ledpin, int fpin, int spin){
	_ledpin = ledpin;
	_fpin = fpin;
	_spin = spin;

	_fval = 0;
	_sval = 0;

	pinMode(_ledpin, OUTPUT);
	pinMode(_fpin, INPUT);
	pinMode(_spin, INPUT);

}

void IRDuo::read(){
	_foff = analogRead(_fpin);
	_soff = analogRead(_spin);
	digitalWrite(_ledpin, HIGH);
	delay(4);
	_fon = analogRead(_fpin);
	_son = analogRead(_spin);
	digitalWrite(_ledpin, LOW);
}

int IRDuo::getf(){
	return _fon-_foff;
}
int IRDuo::gets(){
	return _son-_soff;
}

PT2Val::PT2Val(int led1pin, int led2pin, int ptpin){
	_led1pin = led1pin;
	_led2pin = led2pin;
	_ptpin = ptpin;

	int _valoff = 0;
	int _val1on = 0;
	int _val2on = 0;

	pinMode(_led1pin, OUTPUT);
	pinMode(_led2pin, OUTPUT);
	pinMode(_ptpin, INPUT);
}

void PT2Val::read(){

	_valoff = analogRead(_ptpin);
	digitalWrite(_led1pin, HIGH);
	delay(4);
	_val1on = analogRead(_ptpin);
	digitalWrite(_led1pin, LOW);
	delay(4);
	digitalWrite(_led2pin, HIGH);
	delay(4);
	_val2on = analogRead(_ptpin);
	digitalWrite(_led2pin, LOW);
}

int PT2Val::dif1(){
	return _val1on - _valoff;
}

int PT2Val::dif2(){
	return _val2on - _valoff;
}