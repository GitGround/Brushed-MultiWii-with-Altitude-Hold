
#ifndef Sonar_h
#define Sonar_h

#include "Arduino.h"

class Sonar
{
  public:
    Sonar(int pin);
	uint8_t getDistance();
  private:
    int m_pin;
	const static int MAXIMUM_DISTANCE_LIMIT = 180; //in cm
};

#endif
