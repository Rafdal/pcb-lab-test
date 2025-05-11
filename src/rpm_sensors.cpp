
#include "rpm_sensors.hpp"

static void interruptHandler_S1();
static void interruptHandler_S2();

// Variables para almacenar tiempos
static volatile unsigned long timestamp_S1 = 0;
static volatile unsigned long timestamp_S2 = 0;
static volatile unsigned long delta_S1 = 0;
static volatile unsigned long delta_S2 = 0;

// static volatile bool interruptFlag_S1 = false;
// static volatile bool interruptFlag_S2 = false;

void rpm_sensors_begin(uint8_t S1_PIN, uint8_t S2_PIN)
{
    pinMode(S1_PIN, INPUT); // Pin de interrupción
	pinMode(S2_PIN, INPUT); // Pin de interrupción

    // Configuración de interrupciones
    attachInterrupt(digitalPinToInterrupt(S1_PIN), interruptHandler_S1, FALLING);
    attachInterrupt(digitalPinToInterrupt(S2_PIN), interruptHandler_S2, FALLING);
}

void rpm_sensors_read(unsigned long *s1_delta, unsigned long *s2_delta)
{
    // Leer los valores de delta
    *s1_delta = delta_S1;
    *s2_delta = delta_S2;
    // interruptFlag_S1 = false; // Reinicia la bandera
    // interruptFlag_S2 = false; // Reinicia la bandera
}


static void interruptHandler_S1()
{
    unsigned long ms = millis(); // Captura el tiempo actual
    delta_S1 = ms - timestamp_S1; //
    timestamp_S1 = ms; // Actualiza el tiempo de la última interrupción
    // interruptFlag_S1 = true; // Establece la bandera de interrupción
}

static void interruptHandler_S2()
{
    unsigned long ms = millis(); // Captura el tiempo actual
    delta_S2 = ms - timestamp_S2; //
    timestamp_S2 = ms; // Actualiza el tiempo de la última interrupción
    // interruptFlag_S2 = true; // Establece la bandera de interrupción
}