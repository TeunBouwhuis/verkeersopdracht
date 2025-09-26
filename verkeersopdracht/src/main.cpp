 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <Arduino.h>

enum pinTypes{
    SENSOR,
    COUNTER,
    SEGMENT_DISPLAY,
    SEGMENT_LETTER,
    SEGMENT_DOT,
    TEST,
};

unsigned long curTime = 0;

volatile uint8_t* REG_STORAGE[][3] = {
    {&DDRB, &PORTB, &PINB},
    {&DDRC, &PORTC, &PINC},
    {&DDRD, &PORTD, &PIND},
};

enum registers{
    REG_B,
    REG_C,
    REG_D,
};

const int PINS[][3] = {
    {SENSOR,REG_D,PD2},
    {SENSOR,REG_D,PD3},
    {COUNTER,REG_C,PC0},
    {COUNTER,REG_C,PC1},
    {COUNTER,REG_C,PC2},
    {COUNTER,REG_C,PC3},
    {SEGMENT_LETTER,REG_D,PD4}, // A 
    {SEGMENT_LETTER,REG_D,PD5}, // B 
    {SEGMENT_LETTER,REG_D,PD6}, // C 
    {SEGMENT_LETTER,REG_D,PD7}, // D 
    {SEGMENT_LETTER,REG_B,PB0}, // E
    {SEGMENT_LETTER,REG_B,PB1}, // F 
    {SEGMENT_LETTER,REG_C,PC4}, // G
    {SEGMENT_DISPLAY,REG_B,PB2},
    {SEGMENT_DISPLAY,REG_B,PB3},
    {SEGMENT_DISPLAY,REG_B,PB4},
    {SEGMENT_DISPLAY,REG_B,PB5},          // 0001
    



};