#ifndef GIMBAL_DRV_HPP
#define GIMBAL_DRV_HPP

#include <Arduino.h>

void gimbal_begin(int phA, int phB, int phC, int en, int AS5048_SPI_cs);

void gimbal_run();

#endif // !GIMBAL_DRV_HPP