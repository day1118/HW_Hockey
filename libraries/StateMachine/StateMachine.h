#include "Arduino.h"
#include "Config.h"

class StateMachine
{
public:
	StateMachine(int state, unsigned long stateTime);
 	void setState(int state, unsigned long stateTime);
 	int getState();
 	int getStatePrev();
	unsigned long getTimeSinceChange();
	unsigned long getTimeSinceChangePrev();
	void addTimeSinceChange(unsigned long extraTime);
	void resetTimeSinceChange();
	bool expired();

private:
	int _state;
	unsigned long _changeTime;
	int _statePrev;
	unsigned long _changeTimePrev;
	unsigned long _expireTime;
	bool _neverExpire;
};
