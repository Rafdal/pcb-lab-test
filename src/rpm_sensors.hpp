#ifndef RPM_SEN_HPP
#define RPM_SEN_HPP


#include <Arduino.h>

void rpm_sensors_begin(uint8_t S1_PIN, uint8_t S2_PIN);

void rpm_sensors_read(unsigned long *s1_delta, unsigned long *s2_delta);

bool rpm_sensors_S1_IRQ_flag();
bool rpm_sensors_S2_IRQ_flag();

#endif // RPM_SEN_HPP