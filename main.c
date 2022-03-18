#include <stdio.h>
#include "msp.h"
#include "../inc/Clock.c" // Change these to .h
#include "../inc/Motor.c"
#include "../inc/SysTickInts.c"
#include "FSMController.h"
#include "Reflectance.h"
#include "SensorInt.h"

/**
 * main.c
 */
void main(void)
{
    Clock_Init48MHz();
    SysTick_Init(48000, 5); // Every 1ms, priority 5
    start_fsm();
}
