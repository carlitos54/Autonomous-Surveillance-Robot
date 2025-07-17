/*
Up Arrow:       0010 0000 1101 1111 0000 0010 1111 1101
Left Arrow:     0010 0000 1101 1111 1110 0000 0001 1111
Down Arrow:     0010 0000 1101 1111 1000 0010 0111 1101
Right Arrow:    0010 0000 1101 1111 0110 0000 1001 1111
OK Button:      0010 0000 1101 1111 0010 0010 1101 1101
Power Button:   0010 0000 1101 1111 0001 0000 1110 1111
 */
#include <stdint.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "PWM.h"

#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define UP_ARROW    551486205
#define LEFT_ARROW  551542815
#define DOWN_ARROW  551518845
#define RIGHT_ARROW 551510175
#define OK_BUTTON   551494365
#define POWER       551489775

#define TIME_1 90000    //4tau for a 1
#define TIME_0 45000    //2tau for a 0
#define TIME_13MS 520000
#define TIME_14MS 560000
#define MAX_TIME 2800000    //70ms | Should take 67.5ms

#define REMOTE_MASK 4       //Port D2  WT3CCP0

uint32_t REMOTE = 0;
uint32_t STATE = 0;
uint32_t TIME = 0;
uint32_t PREV_TIME = 0;
uint32_t TOTAL_TIME = 0;
uint32_t CODE = 0;
uint32_t bits = 0;
uint32_t AUTO_MODE = 0;

void initRemote()
{
    GPIO_PORTD_DIR_R &= ~REMOTE_MASK;
    GPIO_PORTD_DEN_R |= REMOTE_MASK;
    GPIO_PORTD_ICR_R |= REMOTE_MASK;            //interrupt clear register
    GPIO_PORTD_IM_R |=  REMOTE_MASK;            //interrupt from pin sent to interrupt controller
    NVIC_EN0_R = 1 << (INT_GPIOD-16);           //turn on interrupt 19 (GPIOD)



    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;           // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 0x4;                          // configure as 32-bit counter (A only)
                                                // configure for capture mode, count up, edge time mode
    WTIMER3_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR;
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_NEG;      // count negative edges
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;            // turn on interrupts for capture //CAEIM Counter A Event Interrupt Mask
    WTIMER3_TAV_R = 0;                          // zero counter for first period
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;            // turn-on counter

}

void Remote_ISR()
{
    TIME = WTIMER3_TAV_R;
    uint32_t TIME_DIFF = (TIME - PREV_TIME);
    GPIO_PORTD_ICR_R |= REMOTE_MASK;
    switch(STATE)
    {
        case 0:                                                             //Idle
            STATE = 1;
            WTIMER3_TAV_R = 0;
            break;
        case 1:                                                 //Preamble Start
            if(TIME_DIFF >= TIME_13MS && TIME_DIFF <= TIME_14MS)          //In between 13ms and 14 ms
            {                                                   //13.5ms = 540000
                STATE = 2;
                PREV_TIME = TIME;
            }
            else
            {
                STATE = 0;
            }
            break;
        case 2:                                                             //Reading Bits
            if(TIME_DIFF <= (TIME_0 + 5000) && TIME_DIFF >= (TIME_0 - 5000))
            {
               REMOTE = REMOTE<<1;
               REMOTE |= 0;
                TOTAL_TIME += TIME_DIFF;
                PREV_TIME = TIME;
            }
            if(TIME_DIFF <= (TIME_1 + 5000) && TIME_DIFF >= (TIME_1 - 5000))
            {
                REMOTE = REMOTE<<1;
                REMOTE |= 1;
                TOTAL_TIME += TIME_DIFF;
                PREV_TIME = TIME;

            }
            bits++;
            if(bits >= 33 || TOTAL_TIME == MAX_TIME)    //if more than 32 bits have been read and more than 69 ms have passed
            {
                STATE = 3;
            }
            break;
        case 3:
            if(REMOTE == OK_BUTTON)
            {
                stop();
                AUTO_MODE = 0;
            }
            if(REMOTE == UP_ARROW)
            {
                stop();
                forward();
                AUTO_MODE = 0;
                BLUE_LED = 0;
            }
            if(REMOTE == DOWN_ARROW)
            {
                stop();
                reverse();
                AUTO_MODE = 0;
                BLUE_LED = 0;
            }
            if(REMOTE == LEFT_ARROW)
            {
                stop();
                counter_clockwise();
                AUTO_MODE = 0;
                BLUE_LED = 0;
            }
            if(REMOTE == RIGHT_ARROW)
            {
                stop();
                clockwise();
                AUTO_MODE = 0;
                BLUE_LED = 0;
            }
            if(REMOTE == POWER)
            {
                stop();
                if(AUTO_MODE == 0)
                {
                    AUTO_MODE = 1;
                }
                else
                {
                    AUTO_MODE = 0;
                    BLUE_LED = 0;
                }

            }

            STATE = 0;
            TIME_DIFF = 0;
            TIME = 0;
            PREV_TIME = 0;
            TOTAL_TIME = 0;
            REMOTE = 0;
            bits = 0;
            WTIMER3_TAV_R = 0;
            break;
    }


}
