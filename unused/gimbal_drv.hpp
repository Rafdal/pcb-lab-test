#ifndef GIMBAL_DRV_HPP
#define GIMBAL_DRV_HPP

#include <Arduino.h>
#include <SimpleFOC.h>

void gimbal_begin(int phA, int phB, int phC, int en);

void gimbal_read();

void gimbal_run();

#endif // !GIMBAL_DRV_HPP