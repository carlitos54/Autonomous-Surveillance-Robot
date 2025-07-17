
#ifndef PIR_ULTRASONIC_H_
#define PIR_ULTRASONIC_H_

#include <stdint.h>
#include <stdlib.h>

extern uint32_t pulseW;
extern uint32_t sonar_Distance;
extern int32_t phase;

void initPIR_ULTRA();
void motion_detect();
void trig_Pulse();
void Timer2A_ISR();
void Echo_ISR();

#endif
