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



uint8_t CollisionData, CollisionFlag;  // mailbox
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
    SysTick_Init(48000, 5); // Every 1ms, priority 5
    Motor_Init();
    BumpInt_Init(&HandleCollision);
    EnableInterrupts();
    start_fsm();

}



