#include "Arduino.h"
#include "Config.h"

class StateMachine
{
public:
	StateMachine(int state, int stateTime);
 	void setState(int state, int stateTime);
 	int getState();
	unsigned long getTimeSinceChange();
	void resetTimeSinceChange();
	bool expired();

private:
	int _state;
	unsigned long _changeTime;
	unsigned long _expireTime;
	bool _neverExpire;
};
