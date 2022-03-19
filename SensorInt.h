// Austin Jadlowiec
// Mar 18 2022
// SensorInt.h
// Reads the reflectance sensors using interrupts

#ifndef SENSORINT_H
#define SENSORINT_H

// Periodically called to handle sensor process
void SysTick_Handler(void);

// Returns the last read data from the sensor
uint8_t ReadSensorData(void);

#endif
