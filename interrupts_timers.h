
#ifndef INTERRUPTS_TIMERS_H_
#define INTERRUPTS_TIMERSH_

#include <stdint.h>
#include <stdlib.h>

#define ADJ_VAL 30

extern int32_t rightW_count;
extern int32_t leftW_count;
extern int32_t ADJ_CountL;
extern int32_t ADJ_CountR;
extern int32_t target_distance;
extern int32_t wheel_distance;
extern uint32_t SPEED_newADJ;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initINTRPT();
void initTimers();
void GPIOC_ISR();
void wheel_correction();
void Timer0A_ISR();
void Timer1A_ISR();
void calculatedDist(int32_t DISTANCE);
void calculateAngle(int32_t ANGLE);

#endif
