// Austin Jadlowiec
// Mar 18 2022
// SensorInt.h
// Reads the reflectance sensors using interrupts

#include <stdint.h>

// [1-9] in ms between
// Delay Start & Delay End
// Change for different reflectivity
const uint8_t READ_DELAY = 1;

uint8_t SensorInput = 0;
uint8_t SensorInput_F = 0; // 0 -> no reading

uint8_t Count = 0;

void SysTick_Handler(void) {

    if (Count % 10 == 0) {
        Reflectance_Start();
    } else if (Count % 10 == READ_DELAY) {
        SensorInput_F = 0;                  // Set Semaphore
        SensorInput = Reflectance_End();    // Write to SensorInput
        SensorInput_F = 1;                  // Reset Semaphore
    }

    Count += 1;
}

uint8_t ReadSensorData(void) {

    // Waiting for SysTick_Handler to write
    while (SensorInput_F);

    return SensorInput;
}
