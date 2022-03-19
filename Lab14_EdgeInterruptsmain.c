// Lab14_EdgeInterruptsmain.c
// Runs on MSP432, interrupt version
// Main test program for interrupt driven bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/BumpInt.h"
#include "../inc/TExaS.h"
#include "../inc/TimerA1.h"
#include "../inc/FlashProgram.h"

//THIS IS THE FUNCTION YOU SHOULD ACTUALLY BE USING
void SysTick_Wait_myVersion(uint32_t delay){

  SysTick->LOAD = (48*delay - 1);// count down to zero
  SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
  while(( SysTick->CTRL&0x00010000) == 0){};
}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P1->OUT &= ~0xC0;
  P2->OUT &= ~0xC0;   // off
  P3->OUT &= ~0xC0;   // low current sleep mode
}

void Motor_InitSimple(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

  // write this as part of Lab 12
  //NOTE: START OF MY CODE

    //====== MOTOR DIRECTIONS: ======
    //Initializing 5.4 Left Motor Direction as output
    //This needs to be output
    P5-> SEL0 &= ~0x10;
    P5-> SEL1 &= ~0x10;
    P5-> DIR |= 0x10;
    P5-> OUT &= ~0x10; //initializing left motor to be forwards (0)
    //Initializing 5.5 Right Motor Direction as output
    P5-> SEL0 &= ~0x20;
    P5-> SEL1 &= ~0x20;
    P5-> DIR |= 0x20;
    P5-> OUT &= ~0x20; //initializing right motor to be forwards (0)


    //====== MOTOR PWML EN: ======
    //Initializing 2.7 Left Motor PWML aka EN as output
    P2-> SEL0 &= ~0x80;
    P2-> SEL1 &= ~0x80;
    P2-> DIR |= 0x80;
    P2-> OUT &= ~0x80; //initializing motor PWM to be off (0)
    //Initializing 2.6 Right Motor PWML aka EN as output
    P2-> SEL0 &= ~0x40;
    P2-> SEL1 &= ~0x40;
    P2-> DIR |= 0x40;
    P2-> OUT &= ~0x40; //initializing motor PWM to be off (0)


    //====== MOTOR nSLEEP: (needs to be 1 for motor to work) ======
    //Initializing 3.7 Left Motor nSLEEP as output
    P3-> SEL0 &= ~0x80;
    P3-> SEL1 &= ~0x80;
    P3-> DIR |= 0x80;
    P3-> OUT |= 0x80; //initializing motor to be on (nSleep=1)
    //Initializing 3.6 Right Motor nSLEEP as output
    P3-> SEL0 &= ~0x40;
    P3-> SEL1 &= ~0x40;
    P3-> DIR |= 0x40;
    P3-> OUT |= 0x40; //initializing motor to be on (nSleep=1)

}

void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    //SETTING UP MOTORS FOR FORWARDS MOTION
    //3.7, Left Motor nSleep = 1
    P3-> OUT |= 0x80;
    //3.6, Right Motor nSleep = 1
    P3-> OUT |= 0x40;
    //5.4, Left Motor Direction = 0
    P5-> OUT &= ~0x10;
    //5.5, Right Motor Direction = 0
    P5-> OUT &= ~0x20;
    uint32_t i=0;
    uint32_t limit=time/10;

    while(i<limit)
    {
        //LEFT MOTOR HIGH AND RIGHT MOTOR HIGH then wait
        P2-> OUT |= 0x80;
        P2-> OUT |= 0x40;
        SysTick_Wait_myVersion(duty);

        //LEFT MOTOR LOW AND RIGHT MOTOR LOW then wait
        P2-> OUT &= ~0x80;
        P2-> OUT &= ~0x40;
        SysTick_Wait_myVersion(10000-duty);
        i++;
    }
    P3->OUT &= ~0xC0;
    //whenever you enter turn 3 on whenever you leave turn 3 off
}



uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   Motor_StopSimple();
}




//To tesT:
/*
 * Go to lab 1 and take motor forward
 * awake outside of while
 * then move inside the while
 * wait for interrupt in thewhile
 */

//
int main(void){  // test of interrupt-driven bump interface
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  CollisionFlag = 0;
  Motor_InitSimple();        // activate Lab 13 software
  BumpInt_Init(&HandleCollision);
  EnableInterrupts();
 // 50%
  while(1){
    Motor_ForwardSimple(7500,10);
    WaitForInterrupt();
  }
}


int main1(void){
  DisableInterrupts();
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock

// write this as part of Lab 14, section 14.4.4 Integrated Robotic System
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}





