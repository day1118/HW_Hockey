#include "Arduino.h"
#include "Config.h"

class Camera
{
public:
	Camera(String name, int pinSI, int pinCLK, int pinAIN);
	int read();
	
private:
	int _pinSI;
	int _pinCLK;
	int _pinAIN;

	String _name;
};