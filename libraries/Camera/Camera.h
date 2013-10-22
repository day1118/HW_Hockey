#include "Arduino.h"
#include "Config.h"

class Camera
{
public:
	Camera(String name, int pinSI, int pinCLK, int pinAIN);
	int read();
	int getCenter();
	int getWidth();
	
private:
	int _pinSI;
	int _pinCLK;
	int _pinAIN;

	String _name;
	int _center;
	int _width;
};