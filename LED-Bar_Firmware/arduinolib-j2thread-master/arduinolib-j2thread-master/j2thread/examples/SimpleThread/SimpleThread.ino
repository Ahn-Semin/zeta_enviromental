#include <j2thread.h>

bool led1=false;
void setup() {
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,INPUT);
  add_thread(new J2ThreadUnit(&loop1));
  add_thread(new J2ThreadUnit(&loop2));
}

bool loop1(J2ThreadUnit* th)
{
  digitalWrite(4, digitalRead(5));
  th->sleep_micro(10);
  return true; 
}

bool loop2(J2ThreadUnit* th)
{
  led1=!led1;
  digitalWrite(3, led1);
  
  th->sleep_milli(500);
  return true; 
}
