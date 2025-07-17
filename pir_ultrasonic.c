
#include <stdint.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "pir_ultrasonic.h"
#include "PWM.h"
#include "remote.h"
#include "wait.h"
#include "uart0.h"

#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define TRIG        (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define PIR         (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

#define TRIG_MASK 1         //PD0
#define ECHO_MASK 64        //PD6
#define PIR_MASK 16          //PE4

uint32_t pulseW = 0;
uint32_t sonar_Distance = 0;
int32_t phase = 0;


void initPIR_ULTRA()
{
    GPIO_PORTD_DIR_R &= ~ECHO_MASK;                     //Input
    GPIO_PORTD_DIR_R |= TRIG_MASK;                      //Output
    GPIO_PORTD_DEN_R |= TRIG_MASK | ECHO_MASK;          //Digital enable
    GPIO_PORTD_AFSEL_R |= ECHO_MASK;
    GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD6_M);
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTE_DIR_R &= ~PIR_MASK;
    GPIO_PORTE_DEN_R |= PIR_MASK;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER2_TAILR_R = 40000000; // set load value to 40e6for 1Hz interrupt rate  time\40MHz
    TIMER2_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts for timeout in timer module
    TIMER2_CTL_R |= TIMER_CTL_TAEN; // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;           // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 0x4;                          // configure as 32-bit counter (A only)
                                                // configure for capture mode, count up, edge time mode
    WTIMER5_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR;
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_BOTH;     // count both edges
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;            // turn on interrupts for capture //CAEIM Counter A Event Interrupt Mask
    WTIMER5_TAV_R = 0;                          // zero counter for first period
    WTIMER5_ICR_R |= TIMER_ICR_CAECINT;
    NVIC_EN3_R = 1 << (INT_WTIMER5A-16-96);     // turn-on interrupt 112 (WTIMER5A)
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;            // turn-on counter


}

void motion_detect()
{
    waitMicrosecond(2000000);                   //waits three second
    if(PIR == 1)
    {
        RED_LED = 1;
        waitMicrosecond(2000000);               //keeps led turned on for 2 seconds
        RED_LED = 0;
    }
    else
    {
        RED_LED = 0;
    }
}

void trig_Pulse()
{
    TRIG = 1;
    waitMicrosecond(10);
    TRIG = 0;
}


void Timer2A_ISR()
{
    trig_Pulse();
    if(sonar_Distance < 80)
    {
        stop();
    }
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;

}


void Echo_ISR()
{
    if(phase == 0)
    {
        WTIMER5_TAV_R = 0;
        phase = 1;
    }
    else
    {
        pulseW = WTIMER5_TAV_R;
        int speedOfSound = 343;
        int sys_clock = 40000000;
        int divisor = (2 * sys_clock)/1000;
        int distance_mm = (pulseW * speedOfSound) / divisor;
        sonar_Distance = distance_mm ;
        phase = 0;
    }
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;
}

