# Autonomous-Surveillance-Robot

## Overview 

This repository contains the code for an autonomous surveillance robot built using the TM4C123GH6PM Tiva C Series microcontroller. The robot can operate in both manual and autonomous modes, utilizing peripherals like PWM for motor control, timers for precise event handling, GPIO interrupts for sensor input, and UART for communication and control. In autonomous mode, the robot uses a PIR sensor to detect motion and an ultrasonic sensor to navigate the environment.

## Features

- Dual-Mode Operation: Can be controlled manually via a remote or UART commands, or can operate autonomously.
- Autonomous Navigation: In autonomous mode, the robot scans its surroundings using an ultrasonic sensor to find the most open area and navigates towards it.
- Motion Detection: A PIR sensor is used to detect motion in the environment.
- Motor Control: Employs PWM for precise control of the robot's two DC motors, allowing for forward, reverse, and turning movements.
- Remote Control: An infrared remote can be used to control the robot's movements and switch between manual and autonomous modes.
- Serial Communication: A shell interface over UART allows for detailed commands to be sent to the robot, such as moving a specific distance or turning by a specific angle.

## Hardware

Microcontroller: TM4C123GH6PM Tiva C Series

Sensors:

* PIR (Passive Infrared) sensor for motion detection.
* Ultrasonic sensor for distance measurement and obstacle avoidance.
* Actuators: Two DC motors with encoders.
* Motor Driver: L298N or similar H-bridge driver.
* Remote: An infrared remote for manual control.

## Software

The project is written in C and is organized into several modules:
- ```main.c```: The main application file. It initializes all hardware components, handles user commands from UART, and implements the state machine for the autonomous surveillance mode.
- ```PWM.c / PWM.h```: Manages the PWM signals for controlling the speed and direction of the two DC motors.
- ```pir_ultrasonic.c / pir_ultrasonic.h```: Contains the drivers and logic for the PIR and ultrasonic sensors. It handles motion detection and distance measurement.
- ```remote.c / remote.h```: Implements the logic for receiving and decoding signals from the infrared remote control.
- ```interrupts_timers.c``` / interrupts_timers.h: Configures and handles all timers and GPIO interrupts. This includes wheel encoder interrupts for distance and angle tracking.
- ```shell.c / shell.h```: Provides a command-line interface over UART to send commands to the robot.
- ```clock.c / clock.h```: Configures the system clock.
- ```wait.c / wait.h```: Provides delay functions.

# Key Functions

## shell.c

```getsUart0```

Gets a character from UART and uses ASCII values to ensure it meets the necessary inputs for the commands defined in main. In the while loop, the function ensures backspaces are ignored, recognizes when a space is entered and ends the string or if a carraige return is sent it ends the string and returns the function.
```c
void getsUart0(USER_DATA *data)
{
    int count = 0;
    while(true)
    {
        char letter = getcUart0();

        if(letter >= 32 && letter != 127)
        {
            data->buffer[count] = letter;
            count++;
            if(count == MAX_CHARS)
            {
                data->buffer[MAX_CHARS] = 0;
                break;
            }
        }
        else if(letter == 8 || letter == 127)
        {
            if(count > 0)
            {
                count--;
            }
        }
        else if(letter == 13)
        {
            data->buffer[count] = 0;
            break;
        }
    }
}
```

```parseFields```

Using 3 sets of characters (alpha, numeric, and delimiter) the function assume the last character was a delimiters when searching the buffer and labels the field according to which character set it falls into, this is done until the end of the buffer string is found or until MAX_FIELDS are reached and returns.
```c
void parseFields(USER_DATA *data)
{
    int counter = 0;
    char prev_char = 'd';
    int i = 0;
    data->fieldCount = 0;
    for(i = 0; data->buffer[i] != '\0'; i++)
    {
        if(data->fieldCount < MAX_FIELDS)
        {
            if((data->buffer[i] > 47 && data->buffer[i] < 58) || data->buffer[i] == 45 || data->buffer[i] == 46)      //numbers
            {
                if(prev_char == 'd')
                {
                    data->fieldPosition[counter] = i;
                    data->fieldCount++;
                    data->fieldType[counter] = 'n';
                    counter++;
                }

                prev_char = 'n';

            }
            else if((data->buffer[i] > 64 && data->buffer[i] < 91) || (data->buffer[i] < 123 && data->buffer[i] > 96))  // alpha
            {
                if(prev_char == 'd')
                {
                    data->fieldPosition[counter] = i;
                    data->fieldCount++;
                    data->fieldType[counter] = 'a';
                    counter++;
                }
                prev_char = 'a';
            }
            else
            {
                data->buffer[i] = '\0';
                prev_char = 'd';
            }

        }
        else
        {
            return;
        }

    }

}
```
```getFieldString```

Returns the value of the field requested if the field number is in range otherwise returns NULL.
```c
char* getFieldString(USER_DATA *data, uint8_t fieldNumber)              //returns pointer to where string exist
{
    char *ret;

    if((fieldNumber <= data->fieldCount || fieldNumber == 0) && data->fieldType[fieldNumber] == 'a')
    {
        ret = &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return NULL;
    }

    return ret;

}
```
```getFieldInteger```

Returns the integer value of the field if the field number is in range and the field type is numeric otherwise returns NULL
```c
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{

    int32_t ret;

    if((fieldNumber <= data->fieldCount || fieldNumber == 0) && data->fieldType[fieldNumber] == 'n')
    {
        ret = atoi(&data->buffer[data->fieldPosition[fieldNumber]]);
    }
    else
    {
        return NULL;
    }

    return ret;

}
```
```isCommand```

This function returns true if the command matches the first field and the number of arguments is greater than or equal to the requested number of minimum arguments.
```c
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    bool ret = 0;
    int counter = 0;
    int pos = data->fieldPosition[0];
    if( (data->fieldType[data->fieldPosition[0]] == 'a') && ( (data->fieldCount - 1) >= minArguments) )
    {
        while(data->buffer[counter] != '\0')
        {
            if(data->buffer[pos] == strCommand[counter])
            {
                counter++;
                pos++;
                ret = 1;
            }
            else
            {
                ret = 0;
                break;
            }
        }
    }
    else
    {
        ret = 0;
    }
    return ret;
}
```
## main.c

```main```

Autonomous Mode State Machine

The core of the autonomous surveillance is a state machine within the ```main()``` function's ```while``` loop. It cycles through three phases: idle, scanning, and moving. While scanning it will use the wheel interrupt counts to calculate and scan several angles to determine the best route, if motion is detected the PIR sensor will trigger a red LED to turn on. 

```c
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
```

Motor Control (```PWM.c```)

The ```forwardD()``` function demonstrates how the robot moves a specific distance at a given speed. It uses the ```calculatedDist()``` function to convert the distance in millimeters to the required number of encoder pulses.

```c
void forwardD(uint32_t SPEED, uint32_t DISTANCE)
{
    DISTANCE_INUSE = 1;
    calculatedDist(DISTANCE);
    forwardS(SPEED);
}
```

Sensor Handling(```pir_ultrasonic.c```)

The ```Echo_ISR()``` is the interrupt service routine for the ultrasonic sensor. It uses a wide timer in capture mode to measure the duration of the echo pulse, which is then converted to a distance in millimeters.

```c
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
```
Remote Control(```remote.c```)

The ```Remote_ISR()``` handles interrupts from the IR receiver. It decodes the received signal to determine which button on the remote was pressed and then executes the corresponding action.

```c
void Remote_ISR()
{
    TIME = WTIMER3_TAV_R;
    uint32_t TIME_DIFF = (TIME - PREV_TIME);
    GPIO_PORTD_ICR_R |= REMOTE_MASK;
    switch(STATE)
    {
        // ... (State machine for decoding IR signal) ...
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
            // ... (other remote button cases) ...
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
            // ... (reset state variables) ...
            break;
    }
}
```
## How to Use

### Hardware Setup:
* Connect Motor Driver inputs to the TM4C123GH6PM 
* (Left Motor:  M1PWM2 (PA6) and M1PWM3 (PA7)
* (Right Motor: M0PWM6 (PC4) and M0PWM7 (PC5)
* Connect the wheel encoder phototransistors to GPIO pins PC6 and PC7.
* Connect the PIR sensor output to pin PE4.
* Connect the ultrasonic sensor:
* Trig pin to PD0.
* Echo pin to PD6.
* Connect the IR receiver output to pin PD2.
* Power the microcontroller and motor driver.

### Software Setup: 
* Open the project in Code Composer Studio.
* Build the project to generate the executable file (.out).
* Flash the .out file to the TM4C123GH6PM.
  
### Running the Robot:
You can control the robot in two ways: through a serial terminal or with an IR remote.

#### UART Commands
Connect to the robots virtual COM port using a terminal emulator (e.g., PuTTY) with a baud rate of 115200. The following commands are available:

Movement (Continuous)
- ```forward```: Moves the robot forward at full speed.
- ```reverse```: Moves the robot in reverse at full speed.
- ```cw```: Rotates the robot clockwise at full speed (±10° degree accuracy).
- ```ccw```: Rotates the robot counter-clockwise at full speed (±10° degree accuracy).
- ```stop```: Stops all motor movement.

Movement (Controlled)

- ```forward [speed] [distance]```: Moves the robot forward for a specific distance (in mm) at a percentage of its max speed (1-100).
- ```reverse [speed] [distance]```: Moves the robot in reverse for a specific distance (in mm) at a percentage of its max speed (1-100).
- ```cw [angle]```: Rotates the robot clockwise by a specific angle in degrees (±10° degree accuracy).
- ```ccw [angle]```: Rotates the robot counter-clockwise by a specific angle in degrees (±10° degree accuracy).

Sensors & Autonomous Mode

- ```ping```: Triggers the ultrasonic sensor and prints the measured distance in millimeters to the terminal.
- ```motion```: Activates the PIR sensor. If motion is detected, the onboard red LED will turn on for two seconds.
- ```on```: Engages the autonomous surveillance mode. The robot will scan its environment, move, and detect motion.
- ```off```: Disengages the autonomous mode.

IR Remote Control

The robot can be controlled using a universal LG TV remote with the following button mapping:

- Up Arrow: Move forward.
- Down Arrow: Move in reverse.
- Left Arrow: Rotate counter-clockwise.
- Right Arrow: Rotate clockwise.
- OK Button: Stop all movement.
- Power Button: Toggle autonomous navigation mode on and off.
