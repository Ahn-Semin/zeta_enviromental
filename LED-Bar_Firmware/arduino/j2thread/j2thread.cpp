#include "J2Thread.h"

J2ThreadUnit::J2ThreadUnit(bool(*callback)(J2ThreadUnit*)) 
{
	this->callback = callback;
}

bool J2ThreadUnit::loop()
{
	return callback(this);
}

void add_thread(J2ThreadUnit* threadunit)
{
	main_thread_list->add_thread(threadunit);
}