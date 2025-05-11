#ifndef RPM_SEN_HPP
#define RPM_SEN_HPP


#include <Arduino.h>

void rpm_sensors_begin(uint8_t S1_PIN, uint8_t S2_PIN);

void rpm_sensors_read(unsigned long *s1_delta, unsigned long *s2_delta);

#endif // RPM_SEN_HPP