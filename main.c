#include <stdio.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/Clock.h" // Change these to .h
#include "../inc/Motor.h"
#include "../inc/SysTickInts.h"
#include "../inc/BumpInt.h"
#include "FSMController.h"
#include "Reflectance.h"
#include "SensorInt.h"



//addition from sensor int started here
const uint8_t READ_DELAY = 1;

uint8_t CollisionData, CollisionFlag;  // mailbox
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
    //while (SensorInput_F);

    return SensorInput;
}

//end of addition from sensro tin


uint8_t getColFlag()
{
    return CollisionFlag;
}


void HandleCollision(uint8_t bumpSensor){
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   Motor_Stop();
}


/**
 * main.c
 */
void main(void)
{

    Clock_Init48MHz();
    SysTick_Init(48000, 2); // Every 1ms, priority 5
    Reflectance_Init();
    EnableInterrupts();
    Motor_Init();
    BumpInt_Init(&HandleCollision);
    start_fsm();

}





