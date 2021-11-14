#ifndef _APP_CONVERT_H_
#define _APP_CONVERT_H_

typedef struct
{
    uint8_t num1;
    uint8_t num2;
} app_convert_dec2Char_t;

/* Convert char to dec */
uint32_t app_convert_char2Dec(uint8_t ch1, uint8_t ch2);
/* Convert dec to char */
app_convert_dec2Char_t app_convert_dec2Char(uint32_t value);

#endif /* _APP_CONVERT_H_ */