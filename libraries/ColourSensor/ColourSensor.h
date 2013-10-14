#include "Arduino.h"

class IRDuo
{
public:
	IRDuo(int ledpin, int fpin, int spin);
	void read();
	int getf();
	int gets();	
//private:
	int _ledpin;
	int _fpin;
	int _spin;

	int _fval;
	int _sval;

	int _foff;
	int _fon;

	int _soff;
	int _son;
};

class PT2Val
{

public:
	PT2Val(int led1pin, int led2pin, int ptpin);
	void read();
	int dif1();
	int dif2();
//private:
	int _led1pin;
	int _led2pin;
	int _ptpin;

	int _valoff;
	int _val1on;
	int _val2on;
};