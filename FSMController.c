#include <stdint.h>

// Linked data structure
struct State {
  int16_t leftDuty;              // Duty Cycle %
  int16_t rightDuty;             // [-100 to 100]
  uint32_t delay;                // Delay in ms
  const struct State *next[8];   // 3-bit input -> 8 next states
};
typedef const struct State State_t;

// Abbreviations:
// OL -> On Line
// L1 -> Off Left 1 (smallest deviation from line)
// NL -> No Line
// LLC -> Left Lost Check (Lost but know we are left of line)
// LC1 -> Lost Check 1 (Very lost)
// LC2 -> Lost Check 2
// FL -> Fully Lost

// State order in &fsm
#define OL &fsm[0]
#define L1 &fsm[1]
#define L2 &fsm[2]
#define L3 &fsm[3]
#define R1 &fsm[4]
#define R2 &fsm[5]
#define R3 &fsm[6]
#define LLC &fsm[7]
#define RLC &fsm[8]
#define LC1 &fsm[9]
#define LC2 &fsm[10]
#define LF &fsm[12]
#define FL &fsm[11]
#define RLLC &fsm[13]
#define RRLC &fsm[14]

// Standard time between states
#define dtGoodGood 40
#define dtGood 30
#define dt 15
#define dtSlightLost 800/2
#define dtLost 800

// Speed PWM definitions (0, 14998)
#define MAX 14998/3  // 100%
#define HIGH 13000/3 // 80%
#define MED 10000/3   // 50%
#define NMED -10000/3 // -50% (Reverse Medium)
#define LOW 5000/3   // 20%


State_t fsm[15]={
  {MAX, MAX,  dtGoodGood, {LC1, L3, L2, L1, OL, R1, R2, R3}},  // On Line
  {MAX, HIGH, dtGood, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 1
  {MAX, MED,  dt, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 2
  {MAX, LOW,  dt, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 3
  {HIGH,MAX,  dtGood, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 1
  {MED, MAX,  dt, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 2
  {LOW, MAX,  dt, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 3
  {MED, NMED, dtSlightLost, {RLLC, L3, L2, L1, OL, R1, R2, R3}},  // Left Lost Check
  {NMED, MED, dtSlightLost, {RRLC, L3, L2, L1, OL, R1, R2, R3}},  // Right Lost Check
  {MED, NMED, dtLost, {LC2, L3, L2, L1, OL, R1, R2, R3}},  // Lost Check 1
  {NMED, MED, dtLost, {LF,  L3, L2, L1, OL, R1, R2, R3}},  // Lost Check 2
  {0, 0,    1000, {FL,  FL, FL, FL, FL, FL, FL, FL}},   // Fully Lost, Stop
  {MED, MED, 650, {FL, L3, L2, L1, OL, R1, R2, R3}}, // Lost forward check //was 700
  {NMED, MED, dtSlightLost, {LC1, L3, L2, L1, OL, R1, R2, R3}},  // Left Lost Check REV
  {MED, NMED, dtSlightLost, {LC1, L3, L2, L1, OL, R1, R2, R3}}  // Right Lost Check REV
};

// Motor Translation Function
// Motor cannot accept negative inputs, only PWM (0 to 14,998)
void call_motor(int16_t leftDuty, int16_t rightDuty) {

    uint8_t leftForward = leftDuty >= 0;
    uint8_t rightForward = rightDuty >= 0;

    // Forward
    if (leftForward && rightForward) {
        Motor_Forward((uint16_t) leftDuty, (uint16_t) rightDuty);

    // Left
    } else if (!leftForward && rightForward) {
        Motor_Left((uint16_t) (-1 * leftDuty), (uint16_t) rightDuty);

    // Right
    } else if (leftForward && !rightForward) {
        Motor_Right((uint16_t) leftDuty, (uint16_t) (-1 * rightDuty));

    // Backward
    } else {
        Motor_Backward((uint16_t) (-1 * leftDuty), (uint16_t) (-1 * rightDuty));
    }
}


State_t *state;

// Starts fsm and will loop continuously
void start_fsm()
{
    // Assume initially on line
    state = OL;

    while(1){
        if(!getColFlag())
        {
            call_motor(state->leftDuty, state->rightDuty);  // Send state output to motor
            Clock_Delay1ms(state->delay);                   // wait
            uint8_t test = ReadSensorData();
            state = state->next[ReadSensorData()];          // next depends on input and state
        }
        else
        {
            state = FL;
        }
    }
}

