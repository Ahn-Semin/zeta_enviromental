#ifndef _J2_THREAD_H
#define _J2_THREAD_H

#include "Arduino.h"
#include "utility/mthread.h"

class J2ThreadUnit : public Thread
{
	public:
		J2ThreadUnit(bool(*callback)(J2ThreadUnit*));
	protected:
		bool loop();
	private:
		bool (*callback)(J2ThreadUnit*);
};

void add_thread(J2ThreadUnit* thread);
#endif