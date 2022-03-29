#ifndef tl
#define tl

#if (ARDUINO >=100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class MyTone  {
  public:
    MyTone(bool displayMsg=false);

    void tone(int pin, int freq, unsigned long duration);

  private:
};
#endif
