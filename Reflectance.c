// Austin Jadlowiec
// Feb 18 2022
// Reflectance.c
// Handles register level sensor reading

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "../inc/Clock.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    // Set up EVEN P5.3
    P5->SEL0 &= ~(0b1 << 3);
    P5->SEL1 &= ~(0b1 << 3);
    P5->DIR |= 0b1 << 3;        // Set as Output
    P5->OUT &= ~(0b1 << 3);     // Initially off

    // Set up ODD P9.2
    P9->SEL0 &= ~(0b1 << 2);
    P9->SEL1 &= ~(0b1 << 2);
    P9->DIR |= 0b1 << 2;        // Set as Output
    P9->OUT &= ~(0b1 << 2);     // Initially off

    // Set up sensor lines P7.0-P7.7
    P7->SEL0 &= ~(0xFF);
    P7->SEL1 &= ~(0xFF);
    P7->DIR &= ~(0xFF);         // Initially inputs
    P7->REN &= ~(0xFF);         // No pull resistor
}

// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // Changing sensors to output
    P7->DIR |= 0xFF;
    P7->OUT &= ~(0xFF);     // Initially off

    // 1) Turn on the IR LEDs
    P5->OUT |= 0b1 << 3;
    P9->OUT |= 0b1 << 2;

    // 2) Initiate a charge the 8 capacitors
    P7->OUT |= 0xFF;

    // 3) Wait for 10us for charging to complete
    Clock_Delay1us(10);

    // 4) Change the directionality of the sensor line ports to make them inputs
    P7->DIR &= ~(0xFF);
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    uint8_t result;

    // Read the binary interpretations of the sensor line voltages
    result = P7->IN & 0xFF;

    // Turn off the IR LEDs
    P5->OUT &= ~(0b1 << 3);
    P9->OUT &= ~(0b1 << 2);

    return AverageSensor(result);
}

// AverageSensor
// Takes sensor data as input and returns
// 3 bit average location of low reflectivity
// 0 -> No data
uint8_t AverageSensor (uint8_t data) {
    uint8_t data_mut = data;
    uint32_t result = 0;
    uint32_t bits = 0;
    uint32_t i;

    for(i = 0; i < 8; i++) {
        result += (data_mut & 1) * (i + 1);
        bits += data_mut & 1;
        data_mut =  data_mut >> 1;
    }

    if (bits == 0) {
        return 0;
    } else {
        return (uint8_t) (result / bits) % 8;
    }
}

