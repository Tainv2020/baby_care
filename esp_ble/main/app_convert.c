#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "app_convert.h"

/* Convert char to dec */
uint32_t app_convert_char2Dec(uint8_t ch1, uint8_t ch2)
{
    uint32_t retVal = 0;

    if((ch1 >= '0') && (ch1 <= '9'))
    {
        retVal = ((ch1 - 48) * 16);
    }
    else if((ch1 >= 'A') && (ch1 <= 'F'))
    {
        retVal = ((ch1 - 55) * 16);
    }
    else if((ch1 >= 'a') && (ch1 <= 'f'))
    {
        retVal = ((ch1 - 87) * 16);
    }

    if((ch2 >= '0') && (ch2 <= '9'))
    {
        retVal += (ch2 - 48);
    }
    else if((ch2 >= 'A') && (ch2 <= 'F'))
    {
        retVal += (ch2 - 55);
    }
    else if((ch2 >= 'a') && (ch2 <= 'f'))
    {
        retVal += (ch2 - 87);
    }

    return retVal; 
}

/* Convert dec to char */
app_convert_dec2Char_t app_convert_dec2Char(uint32_t value)
{
    app_convert_dec2Char_t retVal;

    retVal.num1 = value/16;
    retVal.num2 = value%16;

    return retVal;
}