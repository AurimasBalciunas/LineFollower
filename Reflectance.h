// Austin Jadlowiec
// Feb 18 2022
// Reflectance.h
// Handles register level sensor reading

#ifndef REFLECTANCE_H
#define REFLECTANCE_H

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void);

// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void);


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void);

// AverageSensor
// Takes sensor data as input and returns
// 3 bit average location of low reflectivity
// 0 -> No data
uint8_t AverageSensor (uint8_t data);

#endif
