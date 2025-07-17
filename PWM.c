
#include <stdint.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "interrupts_timers.h"

#define IN1_MASK 16      //Port C4  M0PWM6
#define IN2_MASK 32      //Port C5  M0PWM7
#define IN3_MASK 64      //Port A6  M1PWM2
#define IN4_MASK 128     //Port A7  M1PWM3

int32_t Lcmp = 1023;
int32_t Rcmp = 1000;
int32_t LcmpRev = 1010;
int32_t RcmpRev = 1023;
int32_t FOGpwmL = 0;
int32_t FOGpwmR = 0;
int32_t ROGpwmL = 0;
int32_t ROGpwmR = 0;
uint32_t pwmValL = 0;
uint32_t pwmValR = 0;
uint32_t Rev_pwmValL = 0;
uint32_t Rev_pwmValR = 0;
int32_t PWM1_INUSE = 0;
int32_t PWM2_INUSE = 0;
int32_t PWM3_INUSE = 0;
int32_t PWM4_INUSE = 0;
int32_t ANGLE_INUSE = 0;
int32_t DISTANCE_INUSE = 0;
int32_t SPEED_IN = 0;
int8_t MOTION = 0;


void initPWM()
{
    // Configuration
    GPIO_PORTA_DEN_R |= IN3_MASK | IN4_MASK;
    GPIO_PORTC_DEN_R |= IN1_MASK | IN2_MASK;

    GPIO_PORTA_AFSEL_R |= IN3_MASK | IN4_MASK;
    GPIO_PORTC_AFSEL_R |= IN1_MASK | IN2_MASK;
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA6_M | GPIO_PCTL_PA7_M);  //IN3, IN4
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M);  //IN1, IN2
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA6_M1PWM2 | GPIO_PCTL_PA7_M1PWM3;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7;

    // Configure PWM module 1 to drive RGB LED
    // IN1   on M0PWM6 (PC4), M0PWM3a   Right motor
    // IN2   on M0PWM7 (PC5), M0PWM3b   Right motor
    // IN3   on M1PWM2 (PA6), M1PWM1a   left motor
    // IN4   on M1PWM3 (PA7), M1PWM1b   left motor

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module

    SYSCTL_SRPWM_R = 0;                              // leave reset state

    PWM0_3_CTL_R = 0;                                // turn-off PWM0 generator 3
    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1

    // output 6 on PWM0, gen 3a, cmpa
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
    // output 7 on PWM0, gen 3b, cmpb
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
    // output 2 on PWM1, gen 1a, cmpa
    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    // output 3 on PWM1, gen 1b, cmpb
    PWM1_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;

    PWM0_3_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_1_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;
    PWM1_1_CMPA_R = 0;
    PWM1_1_CMPB_R = 0;

    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 3
    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 1
    PWM0_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN;

                                                     // enable outputs
}

void setPWM(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3, uint16_t PWM4)
{
    PWM0_3_CMPA_R = PWM1;       //Left PC4 Forward
    PWM0_3_CMPB_R = PWM2;       //Left PC5 Reverse
    PWM1_1_CMPA_R = PWM3;       //Right PA6 Forward
    PWM1_1_CMPB_R = PWM4;       //Right PA7 Reverse

}

void forward()
{
    MOTION = 1;
    pwmValL = Lcmp;
    pwmValR = Rcmp;
    SPEED_IN = 100;
    setPWM(pwmValL, 0, pwmValR, 0);
    PWM1_INUSE = 1;
    PWM2_INUSE = 0;
    PWM3_INUSE = 1;
    PWM4_INUSE = 0;
}
//600 mm in 3.79s   158 mm/s    Max Speed 9480
//Lowest Val motor will run is 650
void forwardS(uint32_t SPEED)
{
    MOTION = 1;
    if(SPEED > 100)
    {
        SPEED = 100;
    }
    else if(SPEED < 1)
    {
        SPEED = 0;
    }

    SPEED_IN = SPEED;

    pwmValL = ((Lcmp - 650)/100)*SPEED + 650;
    pwmValR = ((Rcmp - 650)/100)*SPEED + 650;
    FOGpwmL = pwmValL;
    FOGpwmR = pwmValR;
    setPWM(pwmValL, 0 , pwmValR, 0);
}

void forwardD(uint32_t SPEED, uint32_t DISTANCE)
{
    DISTANCE_INUSE = 1;
    calculatedDist(DISTANCE);
    forwardS(SPEED);

}

void reverse()
{
    MOTION = 1;
    pwmValL = LcmpRev;
    pwmValR = RcmpRev;
    SPEED_IN = 100;
    setPWM(0, pwmValL, 0, pwmValR);
    PWM1_INUSE = 0;
    PWM2_INUSE = 1;
    PWM3_INUSE = 0;
    PWM4_INUSE = 1;
}

void reverseS(uint32_t SPEED)
{
    MOTION = 1;
    if(SPEED > 100)
    {
        SPEED = 100;
    }
    else if(SPEED < 1)
    {
        SPEED = 0;
    }

    SPEED_IN = SPEED;

    Rev_pwmValL = ((LcmpRev - 650)/100)*SPEED + 650;
    Rev_pwmValR = ((RcmpRev - 650)/100)*SPEED + 650;
    ROGpwmL = Rev_pwmValL;
    ROGpwmR = Rev_pwmValR;
    setPWM(0, Rev_pwmValL , 0,Rev_pwmValR);



}

void reverseD(uint32_t SPEED, uint32_t DISTANCE)
{
    DISTANCE_INUSE = 1;
    calculatedDist(DISTANCE);
    reverseS(SPEED);
}

void stop()
{
    setPWM(0, 0, 0, 0);
    PWM1_INUSE = 0;
    PWM2_INUSE = 0;
    PWM3_INUSE = 0;
    PWM4_INUSE = 0;
    DISTANCE_INUSE = 0;
    ANGLE_INUSE = 0;
    rightW_count = 0;
    leftW_count = 0;
    ADJ_CountR = 0;
    ADJ_CountL = 0;
    pwmValL = 0;
    pwmValR = 0;
    Rev_pwmValL = 0;
    Rev_pwmValR = 0;
    SPEED_newADJ = 0;
    SPEED_IN = 0;
    FOGpwmL = 0;
    FOGpwmR = 0;
    ROGpwmL = 0;
    ROGpwmR = 0;
    MOTION = 0;
}

void clockwise()
{
    setPWM(0, 1023, 992, 0);
    MOTION = 1;
}

void clockwiseA(int32_t ANGLE)
{
    ANGLE_INUSE = 1;
    calculatedDist(ANGLE);
    clockwise();
}

void counter_clockwise()
{
    setPWM(1023, 0, 0, 992);
    MOTION = 1;

}

void counter_clockwiseA(int32_t ANGLE)
{
    ANGLE_INUSE = 1;
    calculatedDist(ANGLE);
    counter_clockwise();
}

