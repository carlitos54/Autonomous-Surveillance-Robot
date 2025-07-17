
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "PWM.h"

#define PTOUT1_MASK 64   //Port C6  GPIO
#define PTOUT2_MASK 128  //Port C7  GPIO
#define RED_LED_MASK 2   //Port F1
#define BLUE_LED_MASK 4  //Port F2

#define MM_PULSE 14             //mm per pulse
#define WHEEL_C 280             //wheel circumference
#define WHEEL_TO_WHEELC 417     //Wheel to wheel circumference
#define FULL_REV 360            //Full revolution
#define ADJ_VAL 30

int32_t rightW_count = 0;
int32_t leftW_count = 0;
int32_t target_distance = 0;
int32_t wheel_distance = 0;
int32_t ADJ_CountL = 0;
int32_t ADJ_CountR = 0;
uint32_t SPEED_newADJ = 0;

void initINTRPT()
{

    GPIO_PORTC_DIR_R &= ~(PTOUT1_MASK | PTOUT2_MASK);   //Output
    GPIO_PORTC_DEN_R |= PTOUT1_MASK | PTOUT2_MASK;  //Port PC6 and PC7
    GPIO_PORTC_PUR_R |= PTOUT1_MASK | PTOUT2_MASK;


    // Configure falling edge interrupts on PTOUT 1 and 2
    // (edge mode, single edge, falling edge, clear any interrupts, turn on)
    GPIO_PORTC_IS_R  &= ~(PTOUT1_MASK | PTOUT2_MASK);
    GPIO_PORTC_IBE_R &= ~(PTOUT1_MASK | PTOUT2_MASK);
    GPIO_PORTC_IEV_R &= ~(PTOUT1_MASK | PTOUT2_MASK);
    GPIO_PORTC_ICR_R |= PTOUT1_MASK | PTOUT2_MASK;
    GPIO_PORTC_IM_R |=  PTOUT1_MASK | PTOUT2_MASK;
    NVIC_EN0_R = 1 << (INT_GPIOC-16);                // turn-on interrupt 18 (GPIOC)
}

void initTimers()
{
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT;         // configure for one shot
    TIMER0_TAMR_R &= ~TIMER_TAMR_TACDIR;             // count down mode
    TIMER0_TAILR_R = 200000;                         // set load value to 2e5 for 200 Hz interrupt rate
    TIMER0_IMR_R |= TIMER_IMR_TATOIM;                // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER0A-16);              // turn-on interrupt 35

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;      // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT;    // configure for one shot
    TIMER1_TAMR_R &= ~TIMER_TAMR_TACDIR;        // count down mode
    TIMER1_TAILR_R = 200000;                    // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;           // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);         // turn-on interrupt 37

}

void wheel_correction()
{
    if(leftW_count - rightW_count >= 1)          //Left Wheel too fast, slow down left, speed up right
    {                                                //PWM1 Left wheel forward PWM3 Right wheel forward
        //speed used, modify ADJ_VAL                   PWM2 Left wheel reverse PWM4 Right wheel reverse
        if(PWM1_INUSE != 0 && PWM3_INUSE != 0)             //Check if forward PWMs are used
        {
            if(ADJ_CountL < 3)                                  //if adjusted less than 3 times
            {
                if(SPEED_IN == 100)
                {
                    pwmValL -= 3*ADJ_VAL;                            //slow down left, speed up right
                    pwmValR += ADJ_VAL;
                }
                else
                {
                    SPEED_newADJ = (ADJ_VAL *SPEED_IN)/100;
                    pwmValL -= SPEED_newADJ;                            //slow down left, speed up right
                    pwmValR += SPEED_newADJ;
                }

                if(pwmValL > 1023 || pwmValR > 1023 || pwmValL < 1 || pwmValR < 1)  //if values are out of bounds put upper bound
                {
                    if(SPEED_IN == 100)
                    {
                        pwmValL = Lcmp;
                        pwmValR = Rcmp;
                        setPWM(pwmValL, 0 , pwmValR, 0);
                    }
                    else
                    {
                        pwmValL = FOGpwmL;
                        pwmValR = FOGpwmR;
                        setPWM(pwmValL, 0 , pwmValR, 0);
                    }
                }
                else
                {
                    setPWM(pwmValL, 0 , pwmValR, 0);                  //otherwise set new values
                }
                ADJ_CountL++;

            }
            else
            {
                if(SPEED_IN == 100)
                {
                    pwmValL = Lcmp;
                    pwmValR = Rcmp;
                    setPWM(pwmValL, 0 , pwmValR, 0);
                }
                else
                {
                    pwmValL = FOGpwmL;
                    pwmValR = FOGpwmR;
                    setPWM(pwmValL, 0 , pwmValR, 0);
                }
                ADJ_CountL = 0;
            }
        }
        else if(PWM2_INUSE != 0 && PWM4_INUSE != 0)             //Check if reverse PWMs are used
        {
            if(ADJ_CountL < 3)                                  //if adjusted less than 3 times
            {
                if(SPEED_IN == 100)
                {
                    pwmValL -= ADJ_VAL;     //LcmpRev -= ADJ_VAL;                            //slow down right, speed up left
                    //pwmValR += ADJ_VAL;     //Was commented RcmpRev += ADJ_VAL;
                }
                else
                {
                    SPEED_newADJ = (ADJ_VAL *SPEED_IN)/100;
                    pwmValL -= SPEED_newADJ;                            //slow down left, speed up right
                    pwmValR += SPEED_newADJ;
                }
                if(pwmValL > 1023 || pwmValR > 1023 || pwmValL < 1 || pwmValR < 1)              //if values are out of bounds put upper bound
                {
                    if(SPEED_IN == 100)
                    {
                        pwmValL = LcmpRev;
                        pwmValR = RcmpRev;
                        setPWM(0, pwmValL , 0, pwmValR);
                    }
                    else
                    {
                        pwmValL = ROGpwmL;
                        pwmValR = ROGpwmR;
                        setPWM(0, pwmValL , 0, pwmValR);
                    }
                }
                else
                {
                    setPWM(0, pwmValL , 0, pwmValR);              //otherwise set new values
                }
                ADJ_CountL++;
            }
            else
            {
                if(SPEED_IN == 100)
                {
                    pwmValL = LcmpRev;
                    pwmValR = RcmpRev;
                    setPWM(0, pwmValL , 0, pwmValR);
                }
                else
                {
                    pwmValL = ROGpwmL;
                    pwmValR = ROGpwmR;
                    setPWM(0, pwmValL , 0, pwmValR);
                }
                ADJ_CountL = 0;
            }
        }
    }
    else if(rightW_count - leftW_count >= 1)        //Right wheel too fast,speed up left
    {                                               //PWM1 Left wheel forward PWM3 Right wheel forward
        //modify ADJ_VAL using SPEED_IN             //PWM2 Left wheel reverse PWM4 Right wheel reverse
        if(PWM1_INUSE != 0 && PWM3_INUSE != 0)         //Check if forward PWMs are used
        {
            if(ADJ_CountR < 3)                              //if adjusted less than 3 times
            {
                if(SPEED_IN == 100)
                {
                    //pwmValL += ADJ_VAL;                            //slow down right, speed up left
                    pwmValR -= (2 * ADJ_VAL);
                }
                else
                {
                    SPEED_newADJ = (ADJ_VAL *SPEED_IN)/100;
                    pwmValL += SPEED_newADJ;                            //slow down left, speed up right
                    pwmValR -= SPEED_newADJ;
                }
                if(pwmValL > 1023 || pwmValR > 1023 || pwmValL < 1 || pwmValR < 1)              //if values are out of bounds put upper bound
                {
                    if(SPEED_IN == 100)
                    {
                        pwmValL = LcmpRev;
                        pwmValR = RcmpRev;
                        setPWM(pwmValL, 0 , pwmValR, 0);

                    }
                    else
                    {
                        pwmValL = ROGpwmL;
                        pwmValR = ROGpwmR;
                        setPWM(pwmValL, 0 , pwmValR, 0);
                    }
                }
                else
                {
                    setPWM(pwmValL, 0 , pwmValR, 0);              //otherwise set new values
                }
                ADJ_CountR++;
            }
            else
            {
                if(SPEED_IN == 100)
                {
                    pwmValL = Lcmp;
                    pwmValR = Rcmp;
                    setPWM(pwmValL, 0 , pwmValR, 0);
                }
                else
                {
                    pwmValL = ROGpwmL;
                    pwmValR = ROGpwmR;
                    setPWM(pwmValL, 0 , pwmValR, 0);
                }
                ADJ_CountR = 0;
            }
        }
        else if(PWM2_INUSE != 0 && PWM4_INUSE != 0)         //Check if reverse PWMs are used
        {
            if(ADJ_CountR < 3)                              //if adjusted less than 3 times
            {
                if(SPEED_IN == 100)
                {
                    pwmValL += 3;
                    pwmValR -= ADJ_VAL;
                }
                else                           //slow down right, speed up left
                {
                    SPEED_newADJ = (ADJ_VAL *SPEED_IN)/100;
                    pwmValL += SPEED_newADJ;                            //slow down left, speed up right
                    pwmValR -= SPEED_newADJ;

                }
                if(pwmValL > 1023 || pwmValR > 1023 || pwmValL < 1 || pwmValR < 1)              //if values are out of bounds put upper bound
                {
                    if(SPEED_IN == 100)
                    {
                        pwmValL = LcmpRev;
                        pwmValR = RcmpRev;
                        setPWM(0, pwmValL , 0, pwmValR);
                    }
                    else
                    {
                        pwmValL = ROGpwmL;
                        pwmValR = ROGpwmR;
                        setPWM(0, pwmValL , 0, pwmValR);
                    }

                }
                else
                {
                    setPWM(0, pwmValL , 0, pwmValR);              //otherwise set new values
                }
                ADJ_CountR++;
            }
            else
            {
                if(SPEED_IN == 100)
                {
                    pwmValL = LcmpRev;
                    pwmValR = RcmpRev;
                    setPWM(0, pwmValL , 0, pwmValR);
                }
                else
                {
                    pwmValL = ROGpwmL;
                    pwmValR = ROGpwmR;
                    setPWM(0, pwmValL , 0, pwmValR);
                }
                ADJ_CountR = 0;
            }
        }
    }
    else
    {
        if(PWM1_INUSE != 0 && PWM3_INUSE != 0)
        {
            setPWM(pwmValL, 0, pwmValR, 0);                    //Forward settings
        }
        else if(PWM2_INUSE != 0 && PWM4_INUSE != 0)
        {
            setPWM(0, pwmValL, 0, pwmValR);                    //Reverse settings
        }
    }
}

void GPIOC_ISR(void)                        //Right Wheel
{
    if(GPIO_PORTC_MIS_R & PTOUT1_MASK)
    {
        rightW_count++;
        GPIO_PORTC_IM_R &= ~(PTOUT1_MASK);
        GPIO_PORTC_ICR_R |= PTOUT1_MASK;
        TIMER0_CTL_R = TIMER_CTL_TAEN;
    }

    if(GPIO_PORTC_MIS_R & PTOUT2_MASK)      //Left Wheel
    {
        leftW_count++;
        GPIO_PORTC_IM_R &= ~(PTOUT2_MASK);
        GPIO_PORTC_ICR_R |= PTOUT2_MASK;
        TIMER1_CTL_R = TIMER_CTL_TAEN;
    }

    if(leftW_count >= 2 && rightW_count >= 2)       //((leftW_count + rightW_count)/2) used for distance
    {
        wheel_correction();
    }

    if(target_distance != 0 && rightW_count >= target_distance && leftW_count >= target_distance )
    {
        stop();
        target_distance = 0;
    }

}




void Timer0A_ISR()
{
    GPIO_PORTC_IM_R |= PTOUT1_MASK;
    GPIO_PORTC_ICR_R |= PTOUT1_MASK;
    TIMER0_ICR_R |= TIMER_ICR_TATOCINT;
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
}


void Timer1A_ISR()                  //Left Wheel
{
    GPIO_PORTC_IM_R |= PTOUT2_MASK;
    GPIO_PORTC_ICR_R |= PTOUT2_MASK;
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
}

// Circumference of wheel = 89 * pi = 280 mm
//20 holes 280mm/20 holes = 14 mm per hole
//Circumference from wheel to wheel: 133 * pi = 417 mm
//14 mm per hole
//error margin 10 degrees
void calculatedDist(int32_t DISTANCE)
{
            //converting distance in mm to target distance using holes
    if(DISTANCE_INUSE == 1)
    {
        int32_t comp = 0;
        if(DISTANCE <= 300)
        {
            comp = 0;
        }
        else if(DISTANCE <= 600)
        {
            comp = 0;
        }
        else if(DISTANCE <= 1200)
        {
            comp = 0;
        }
        else
        {
            comp = (DISTANCE - 300)/250;
        }
        target_distance = (DISTANCE / MM_PULSE) + (comp/2);
    }

    if(ANGLE_INUSE == 1)
    {
        int32_t comp = 0;
        if(DISTANCE < 180)
        {
            comp = 0;
        }
        else if(DISTANCE < 270 && DISTANCE >= 180)
        {
            comp = (DISTANCE - 90)/60;
        }
        else if(DISTANCE <= 360 && DISTANCE >= 270)
        {
            comp = (DISTANCE - 180)/60;
        }
        else
        {
            comp = (DISTANCE - 180)/65;
        }
        target_distance = (DISTANCE / MM_PULSE) + comp;
    }
}

/*
void calculateAngle(int32_t ANGLE)
{
    wheel_distance = (ANGLE/ FULL_REV)/ WHEEL_TO_WHEELC;     //converting distance in mm to target distance needed for angle
}
*/



