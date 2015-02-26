#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(int pin)
{
  m_pin = pin;
}

uint8_t Sonar::getDistance()
{
	long analogVolt = analogRead(m_pin);
	uint8_t distance = (analogVolt*1.27); //Convert to analog Voltage to cm Distance
	return distance = (distance > MAXIMUM_DISTANCE_LIMIT)? MAXIMUM_DISTANCE_LIMIT : distance; //Limit distance reading to MAXIMUM_DISTANCE_LIMIT

	//uint8_t distance = (pulseIn(m_pin, HIGH))*0.017; //147uS per inch // then change inches to centimeters // for PWM reading
}

