

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdlib.h>

extern int32_t Lcmp;
extern int32_t Rcmp;
extern int32_t LcmpRev;
extern int32_t RcmpRev;
extern int32_t FOGpwmL;
extern int32_t FOGpwmR;
extern int32_t ROGpwmL;
extern int32_t ROGpwmR;
extern int32_t SPEED_IN;
extern uint32_t pwmValL;
extern uint32_t pwmValR;
extern int32_t PWM1_INUSE;
extern int32_t PWM2_INUSE;
extern int32_t PWM3_INUSE;
extern int32_t PWM4_INUSE;
extern int32_t DISTANCE_INUSE;
extern int32_t ANGLE_INUSE;
extern uint32_t Rev_pwmValL;
extern uint32_t Rev_pwmValR;
extern int8_t MOTION;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initPWM();
void setPWM(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3, uint16_t PWM4);
void forward();
void forwardS(uint32_t SPEED);
void forwardD(uint32_t SPEED, uint32_t DISTANCE);
void reverse();
void reverseS(uint32_t SPEED);
void reverseD(uint32_t SPEED, uint32_t DISTANCE);
void stop();
void clockwise();
void clockwiseA(int32_t ANGLE);
void counter_clockwise();
void counter_clockwiseA(int32_t ANGLE);

#endif
