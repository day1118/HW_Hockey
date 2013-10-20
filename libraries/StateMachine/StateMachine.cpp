#include "StateMachine.h"

StateMachine::StateMachine(int state, int stateTime) {
	_state = state;
	_changeTime = millis();
	if(stateTime == NEVER_EXPIRE)
	{
		_neverExpire = true;	
	}
	else
	{
		_expireTime = millis() + stateTime;	
		_neverExpire = false;
	}
	
}

void StateMachine::setState(int state, int stateTime){
	if(_state != state)
	{
		_state = state;
		_changeTime = millis();
	}

	if(stateTime == NEVER_EXPIRE)
		_neverExpire = true;
	else
	{
		_expireTime = millis() + stateTime;
		_neverExpire = false;
	}

}

int StateMachine::getState(){
	return _state;
}
unsigned long StateMachine::getTimeSinceChange(){
	return millis() - _changeTime;
}

void StateMachine::resetTimeSinceChange(){
	_changeTime = millis();
}	

bool StateMachine::expired(){
	if(_neverExpire)
		return false;
	else
		return _expireTime < millis();
}
