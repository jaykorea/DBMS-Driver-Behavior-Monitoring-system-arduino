#include "NewToneLib.h"

MyTone::MyTone(bool displayMsg) {
  // Anything you need when instantiating your object goes here
  //_msg = displayMsg;
}

void MyTone::tone(int pin, int freq, unsigned long duration)
{

  //freq in Hz; min = 31Hz; max = 333Hz
  //duration in milliseconds; max = 4290000‬ms
  //duty cycle is 50%
  //pin shoud be declared as output in setup
  if (freq >= 31 && freq <= 333333 && duration > 0 && duration <= 4290000)
  {
    duration *= 1000; //duration in microseconds
    int halfT = (int)(float(1000000) / float(2 * freq) + 0.5); //half period in microseconds
    if (2 * halfT > duration)
      return;
    unsigned long cycles = (unsigned long)(float(duration) / float(2 * halfT) + 0.5);
    for (unsigned long i = 0; i < cycles; i++)
    {
      digitalWrite(pin, 1);
      delayMicroseconds(halfT);
      digitalWrite(pin, 0);
      delayMicroseconds(halfT);
    }
  }
}

