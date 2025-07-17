

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "uart0.h"
#include "shell.h"
#include "PWM.h"
#include "interrupts_timers.h"
#include "remote.h"
#include "pir_ultrasonic.h"
#include "wait.h"
#include "tm4c123gh6pm.h"

//Bitbanding
#define SLEEP       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

// Masks
#define SLEEP_MASK 16    //Port F4  GPIO
#define RED_LED_MASK 2   //Port F1
#define BLUE_LED_MASK 4  //Port F2
#define ANGLE_CHECK 90

uint32_t PHASE = 0;
uint32_t best_dist = 0;
uint32_t best_angle = 0;
uint32_t temp_dist = 0;
uint32_t temp_angle = 0;

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    //Port A, C, D, E, F clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R4| SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R0;
    //M0 and M1 clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0 | SYSCTL_RCGCPWM_R1;
    //Timers
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2 | SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R0;
    //WTimer 3
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R3 | SYSCTL_RCGCWTIMER_R5;
    _delay_cycles(3);

    GPIO_PORTF_DIR_R |= SLEEP_MASK | RED_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |= SLEEP_MASK | RED_LED_MASK | BLUE_LED_MASK;
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)

{
    USER_DATA data;

    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initPWM();
    initINTRPT();
    initTimers();
    initRemote();
    SYSCTL_SRUART_R |= SYSCTL_SRUART_R0;
    initPIR_ULTRA();
    SLEEP = 1;
    AUTO_MODE = 0;
    while(true){
        if(kbhitUart0())
        {
            BLUE_LED = 0;
            AUTO_MODE = 0;
            bool valid = false;
            getsUart0(&data);
            parseFields(&data);
            if (isCommand(&data, "forward", 0))
            {
                valid = true;
                rightW_count = 0;
                leftW_count = 0;
                if(data.fieldCount == 3)
                {
                    int32_t SPEED = getFieldInteger(&data, 1);
                    int32_t DISTANCE = getFieldInteger(&data, 2);

                    forwardD(SPEED, DISTANCE);

                }
                else if(data.fieldCount == 2)
                {
                    int32_t SPEED = getFieldInteger(&data, 1);
                    forwardS(SPEED);
                }
                else
                {
                    forward();
                }

            }
            else if(isCommand(&data, "reverse", 0))
            {
                valid = true;
                rightW_count = 0;
                leftW_count = 0;
                if(data.fieldCount == 3)
                {
                    int32_t SPEED = getFieldInteger(&data, 1);
                    int32_t DISTANCE = getFieldInteger(&data, 2);

                    reverseD(SPEED, DISTANCE);

                }
                if(data.fieldCount == 2)
                {
                     int32_t SPEED = getFieldInteger(&data, 1);
                     reverseS(SPEED);
                }
                else
                {
                     reverse();
                }

            }
            else if(isCommand(&data, "cw", 0))
            {
                valid = true;
                rightW_count = 0;
                leftW_count = 0;
                if(data.fieldCount == 2)
                {
                    int32_t ANGLE = getFieldInteger(&data, 1);
                    clockwiseA(ANGLE);
                }
                else
                {
                    clockwise();
                }

            }
            else if(isCommand(&data, "ccw", 0))
            {
                valid = true;
                rightW_count = 0;
                leftW_count = 0;
                if(data.fieldCount == 2)
                {
                    int32_t ANGLE = getFieldInteger(&data, 1);
                    counter_clockwiseA(ANGLE);
                }
                else
                {
                    counter_clockwise();
                }

            }
            else if(isCommand(&data, "stop", 0))
            {
                valid = true;
                stop();
            }
            else if (isCommand(&data, "navigate", 1))
            {
                char* str = getFieldString(&data, 1);
                valid = true;
                if(strcompare(str, "on") || strcompare(str, "ON"))
                {
                    AUTO_MODE = 1;
                }
                else if(strcompare(str, "off") || strcompare(str, "OFF"))
                {
                    if(AUTO_MODE == 1)
                    {
                        stop();
                    }
                    AUTO_MODE = 0;
                }
                else
                {
                    putsUart0("Invalid Field");
                    putcUart0('\n');
                }
            }

            else if(isCommand(&data, "ping", 0))
            {
                valid = true;
                trig_Pulse();
                intToString(sonar_Distance);
                sonar_Distance = 0;
            }
            else if(isCommand(&data, "motion", 0))
            {
                valid = true;
                motion_detect();
                stop();
            }
            else if(!valid)
            {
                putsUart0("Invalid command\n");
            }
        }

        if(AUTO_MODE == 1)
        {

            BLUE_LED = 1;
            switch(PHASE)
            {
                case 0:                                 //IDLE
                    //turn off timer
                    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
                    PHASE = 1;
                    best_dist = 0;
                    best_angle = 0;
                    temp_dist = 0;
                    temp_angle = 0;
                    sonar_Distance = 0;
                    break;
                case 1:
                    stop();
                    if(AUTO_MODE == 0) break;
                    int i;
                    for(i = 1; i < 5; i++)
                    {
                        if(AUTO_MODE == 0) break;
                        motion_detect();
                        trig_Pulse();
                        temp_dist = sonar_Distance;
                        temp_angle = i * ANGLE_CHECK;
                        if(temp_dist > best_dist)
                        {
                            best_dist = temp_dist;
                            best_angle = temp_angle;
                            clockwiseA(ANGLE_CHECK);
                        }
                        else
                        {
                            temp_dist = 0;
                            temp_angle = 0;
                            clockwiseA(ANGLE_CHECK);
                        }
                    }
                    clockwiseA(best_angle);
                    PHASE = 2;
                    break;
                case 2:
                    //turn on timer to continuously ping
                    //ping distance in timer, check if distance less than 300 too close to wall
                    //go back to state 0 to zero everything
                    TIMER2_CTL_R |= TIMER_CTL_TAEN;
                    if(AUTO_MODE == 0) break;
                    forwardD(100, 1500);
                    PHASE = 0;
                break;
            }

        }

    }
}
