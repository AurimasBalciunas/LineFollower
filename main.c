#include <stdio.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/Clock.h"
#include "../inc/Motor.h"
#include "../inc/SysTickInts.h"
#include "../inc/BumpInt.h"
#include "../inc/FlashProgram.h"
#include "FSMController.h"
#include "Reflectance.h"
#include "SensorInt.h"

const uint8_t READ_DELAY = 1;
uint8_t CollisionData, CollisionFlag;  // mailbox
uint8_t SensorInput = 0;
uint8_t SensorInput_F = 0; // 0 -> no reading
uint8_t Count = 0;

void SysTick_Handler(void){

    if (Count % 10 == 0) {
        Reflectance_Start();
    } else if (Count % 10 == READ_DELAY) {
        SensorInput_F = 0;                  // Set Semaphore
        SensorInput = Reflectance_End();    // Write to SensorInput
        SensorInput_F = 1;                  // Reset Semaphore
    }

    Count += 1;
}

uint8_t ReadSensorData(void){

    // Waiting for SysTick_Handler to write
    //while (SensorInput_F);

    return SensorInput;
}


//Accessor Function for FSM Controller to stop operating if collision has occured.
uint8_t getColFlag(){
    return CollisionFlag;
}


void HandleCollision(uint8_t bumpSensor){
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   Motor_Stop();
}


uint8_t DataLogging_A = 1;

// Erase Flash ROM (Set to 0xFF)
void Debug_FlashInit(){
  uint32_t flashAddr = 0x00020000;
  while (flashAddr < 0x0003FFFF)
  {
      Flash_Erase(flashAddr);
      flashAddr += 256;
  }
}


/**
 * main.c
 */
void main(void){
    Clock_Init48MHz();
    SysTick_Init(48000, 2); // Every 1ms, priority 2
    Reflectance_Init();
    EnableInterrupts();
    Motor_Init();
    BumpInt_Init(&HandleCollision); //Pass is not necessary in here since we call the function directly
    if (DataLogging_A == 1) { Debug_FlashInit(); }
    start_fsm();
}





