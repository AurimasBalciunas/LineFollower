#ifndef FSMCONTROLLER_H
#define FSMCONTROLLER_H

/* Motor Translation Function
 * Motor cannot accept negative inputs, only PWM (0 to 14,998)
 * Decides which function to call based on signs of R/L
 */
void call_motor(int16_t leftDuty, int16_t rightDuty);

// Initializes Finite State Machine and continues looping indefinitely
void start_fsm();

#endif
