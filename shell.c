// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

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

bool strcompare(const char str1[], const char str2[])
{
    while(*str1 != '\0' && *str2 != '\0')
    {
        if(*str1 != *str2)
        {
            return false;
        }
        str1++;
        str2++;
    }
    if(*str1 == '\0' && *str1 == '\0')
    {
        return true;
    }
    else
    {
        return false;
    }
}

void intToString(int val)
{
    char str[5];
    int thousand = val/1000;
    if(thousand >= 0)
    {
        char temp = '0' + thousand;
        val -= (thousand * 1000);
        str[0] = temp;
    }
    int hundred = val/100;
    if(hundred >= 0)
    {
        char temp = '0' + hundred;
        val -= (hundred * 100);
        str[1] = temp;
    }
    int ten = val/10;
    if(ten >= 0)
    {
        char temp = '0' + ten;
        val -= (ten * 10);
        str[2] = temp;
    }
    int one = val/1;
    if(one >= 0)
    {
        char temp = '0' + one;
        val -= (one * 1);
        str[3] = temp;
    }
    str[4] = '\0';
    putsUart0(str);

}

