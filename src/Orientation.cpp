#include "Orientation.h"

// Testing Offset
//float offsetX = -38.500000;
//float offsetY = 23.500000;

// Devuelve en formato float la orientacion del CANSAT en grados [0, 360]°
float computeHeading(float &x, float &y, float &offsetX, float &offsetY) {
  float heading = atan2(y-offsetY, x-offsetX)*180.0 / M_PI + 90; // En grados
        if (heading < 0) heading += 360.0;
  return heading;
}

