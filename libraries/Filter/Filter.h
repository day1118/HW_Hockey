#ifndef FILTER_H
	#define FILTER_H

	#include "Arduino.h"
	#include "Config.h"

	class Filter{
	public:
		Filter(int size);
		void update(bool newState);
		bool on();
		unsigned long getTimeSinceChange();
		void resetTimeSinceChange();

	private:
		int _counter;
		bool _active;
		int _size;
		unsigned long _changeTime;
	};

#endif