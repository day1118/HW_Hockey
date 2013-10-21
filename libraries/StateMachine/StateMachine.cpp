#include "StateMachine.h"

StateMachine::StateMachine(int state, unsigned long stateTime) {
	_state = state;
	_changeTime = millis();

	_statePrev = state;
	_changeTimePrev = millis();

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

void StateMachine::setState(int state, unsigned long stateTime){
	if(_state != state)
	{
		_statePrev = _state;
		_changeTimePrev = _changeTime;

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

int StateMachine::getStatePrev(){
	return _statePrev;
}

unsigned long StateMachine::getTimeSinceChange(){
	return millis() - _changeTime;
}

unsigned long StateMachine::getTimeSinceChangePrev(){
	return millis() - _changeTimePrev;
}

void StateMachine::addTimeSinceChange(unsigned long extraTime){
	_changeTime = _changeTime - extraTime;
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
