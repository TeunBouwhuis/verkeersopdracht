 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <Arduino.h>

 #define START 0
 #define DDR_X 0
 #define PORT_X 1
 #define PIN_X 2

 #define PIN_FUNCTION 0
 #define PIN_REGISTER 1
 #define PIN_PIN 2



 #define ACTIVATION_INTERVAL 8000

 #define TASK 0
//  0 = 0111111 = 63
//  1 = 0000110 = 6
//  2 = 1011011 = 91
//  3 = 1001111 = 79
//  4 = 1100110 = 102
//  5 = 1101101 = 109
//  6 = 1111101 = 125
//  7 = 0000111 = 7
//  8 = 1111111 = 127
//  9 = 1101111 = 111

int evilArray[] = {63,6,91,79,102,109,125,7,127,111};
uint8_t currentCount = 0;



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

ensor* Sensors = nullptr;
uint8_t maxSensors = 0;

struct Counter{
    const int* pointer;
};

Counter* Counters = nullptr;
uint8_t maxCounters = 0;
uint32_t maxCounterValue = 1;

struct Display{
    const int* pointer;
};

struct Display* Displays = nullptr;
uint8_t maxDisplays = 0;

struct DisplayC{
    const int* pointer;
};
int digits[4];
uint8_t digitAmount = 0;

struct DisplayC* DisplayCs = nullptr;
uint8_t maxDisplayCs = 0;

void button_state(uint8_t proposedState, uint8_t sensorIndex){                 
    if ((Sensors[sensorIndex].currentState != proposedState) && (millis() - Sensors[sensorIndex].lastPressTime > debounceTime)) {
        Sensors[sensorIndex].lastPressTime = millis();
        Serial.println(proposedState);
        if (proposedState == PRESSED) {
            Sensors[sensorIndex].lastActivation = millis(); // Als hij gedrukt is kan vehicle_passed dat zien
        }
        
        Sensors[sensorIndex].currentState = (Sensors[sensorIndex].currentState == PRESSED) ? RELEASED : PRESSED;
    }
}

void axle_detected(){
    for (uint8_t CURRENT_SENSOR = START; CURRENT_SENSOR < maxSensors; CURRENT_SENSOR++){
        const int* toCheck = Sensors[CURRENT_SENSOR].pointer;
                 //bool pinValue = (PIND & (1 << PD2)) != 0;
  //Serial.println(pinValue);

        button_state(*REG_STORAGE[toCheck[PIN_REGISTER]][PIN_X] >> toCheck[PIN_PIN] & 1,CURRENT_SENSOR);
    }
}

bool vehicle_passed(){
    axle_detected();

    unsigned long activationTime = millis() - Sensors[START].lastActivation;

    if ((millis() - Sensors[START].lastActivation) < ACTIVATION_INTERVAL) {
        unsigned long startTime = Sensors[START].lastActivation;
        unsigned long timeLeft = ACTIVATION_INTERVAL - (millis() - startTime);        
        unsigned long finalSensorTime = startTime; 
    
        for(uint8_t CURRENT_SENSOR = 0; CURRENT_SENSOR < maxSensors; CURRENT_SENSOR++) {
            if (CURRENT_SENSOR == START) continue;        
            if (Sensors[CURRENT_SENSOR].lastActivation > startTime) {
                unsigned long sensorDelay = Sensors[CURRENT_SENSOR].lastActivation - startTime;
                if (sensorDelay > timeLeft) {
                    return false;
                }
                timeLeft = ACTIVATION_INTERVAL - sensorDelay;
                finalSensorTime = Sensors[CURRENT_SENSOR].lastActivation; // breh
            } else {
                return false;
            }
        }
        
        total_time = finalSensorTime - startTime;
        return true;
    }
    return false;
}
void InitializeIO(){
    // zet elke eerst register op 0 omdat arduino soms spookt. 
    for (uint8_t CURRENT_REGISTER = START; CURRENT_REGISTER < sizeof(REG_STORAGE) / sizeof(REG_STORAGE[START]); CURRENT_REGISTER++ )
    { 
        for (uint8_t CURRENT_INDEX = START; CURRENT_INDEX < sizeof(REG_STORAGE[START]) / sizeof(REG_STORAGE[START][START]); CURRENT_INDEX++){
            *REG_STORAGE[CURRENT_REGISTER][CURRENT_INDEX] = 0;
        }
    }

    for (uint8_t CURRENT_PIN = START; CURRENT_PIN < sizeof(PINS) / sizeof(PINS[0]); CURRENT_PIN++)
    {
        uint8_t givenTask = PINS[CURRENT_PIN][0];
        switch (givenTask)
        {
        case SENSOR:
            maxSensors++;
            break;
        case COUNTER:
            maxCounters++;
            break;
        case SEGMENT_LETTER:
            maxDisplays++;
            break;
        case SEGMENT_DISPLAY:
            maxDisplayCs++;
            break;
        default:
            break;
        }
    }

    for(int32_t loop = START; loop < maxCounters; loop++) { // 2^aantal leds pakken voor de counter
    maxCounterValue *= 2;
    }
    maxCounterValue += -1;

    Serial.println(maxCounterValue);

    Sensors = new Sensor[maxSensors];
    Counters = new Counter[maxCounters];
    Displays = new Display[maxDisplays];
    DisplayCs = new DisplayC[maxDisplayCs];
    uint8_t counterIndex = 0;
    uint8_t sensorIndex = 0;
    uint8_t displayIndex = 0;
    uint8_t displayCIndex = 0;


    // Setup pins gebaseerd op eerder aangewezen functie.
    for (uint8_t CURRENT_PIN = START; CURRENT_PIN < sizeof(PINS) / sizeof(PINS[0]); CURRENT_PIN++) 
    {
        uint8_t givenTask = PINS[CURRENT_PIN][0];
        uint8_t pinRegisters = PINS[CURRENT_PIN][1];
        uint8_t pinValue = PINS[CURRENT_PIN][2];
        Serial.println("im here dude");

        switch (givenTask)
        {
        case TEST: // Zet gwn aan lol
            *REG_STORAGE[pinRegisters][DDR_X] |= (1 << pinValue);
            *REG_STORAGE[pinRegisters][PORT_X] |= (1 << pinValue); 
            break;
        case SENSOR: // Zet op input met interne pull up. 
            *REG_STORAGE[pinRegisters][DDR_X] &= ~(1 << pinValue);
            *REG_STORAGE[pinRegisters][PORT_X] |= (1 << pinValue);

            Sensors[sensorIndex].lastPressTime = 0;
            Sensors[sensorIndex].currentState = RELEASED;
            Sensors[sensorIndex].pointer = PINS[CURRENT_PIN];
            Sensors[sensorIndex].lastActivation = 0;
            sensorIndex++;
            break;
        case COUNTER: // Zet op output
            *REG_STORAGE[pinRegisters][DDR_X] |= (1 << pinValue);
            Counters[counterIndex].pointer = PINS[CURRENT_PIN];
            counterIndex++;
            break;
        case SEGMENT_LETTER:
            *REG_STORAGE[pinRegisters][DDR_X] |= (1 << pinValue);
            *REG_STORAGE[pinRegisters][PORT_X] |= (1 << pinValue); 
            Displays[displayIndex].pointer = PINS[CURRENT_PIN];
            displayIndex++;
            break;
        case SEGMENT_DISPLAY:
            *REG_STORAGE[pinRegisters][DDR_X] |= (1 << pinValue);
            DisplayCs[displayCIndex].pointer = PINS[CURRENT_PIN];
            displayCIndex++;
            break;
        default:
            break;
        }
    }
    *REG_STORAGE[0][PORT_X] |= (1 << PB3);
    *REG_STORAGE[1][0] |= (1 << PC5);
    *REG_STORAGE[2][0] |= (1 << PD1);
    *REG_STORAGE[2][1] &= ~(1 << PD1);
}

void display_counter(uint32_t count){
    for(int COUNTER_INDEX = START; COUNTER_INDEX < maxCounters; COUNTER_INDEX++) {
        if(count & (1 << COUNTER_INDEX)) {
        *REG_STORAGE[Counters[COUNTER_INDEX].pointer[PIN_REGISTER]][PORT_X] |= (1 << Counters[COUNTER_INDEX].pointer[PIN_PIN]); 
        } else {
        *REG_STORAGE[Counters[COUNTER_INDEX].pointer[PIN_REGISTER]][PORT_X] &= ~(1 << Counters[COUNTER_INDEX].pointer[PIN_PIN]); 
        }
    }
}

uint8_t get_size(uint32_t number){
    int numDigits = 0;
    int tempCount = number;
        while(tempCount != 0) {
            tempCount /= 10;
            numDigits++;
    }
    return numDigits;
}

void get_digits(uint32_t number){
    digitAmount = get_size(number);
    int temp = number;
    for(int i = digitAmount - 1; i >= 0; i--) {
        digits[i] = temp % 10;
        temp = temp / 10;
}
}



void display_speed() {
    //weeooweewoo
    int displayDigits[digitAmount];
    for(int i = 0; i < digitAmount; i++) {
        displayDigits[i] = digits[digitAmount - 1 - i];
    }
    
    for (size_t CURRENT_DISPLAY = 0; CURRENT_DISPLAY < digitAmount; CURRENT_DISPLAY++) {
        // alles uit
        for (size_t i = 0; i < maxDisplayCs; i++) {
            *REG_STORAGE[DisplayCs[i].pointer[PIN_REGISTER]][PORT_X] |= (1 << DisplayCs[i].pointer[PIN_PIN]);
        }
        
        // goeie aan
        *REG_STORAGE[DisplayCs[CURRENT_DISPLAY].pointer[PIN_REGISTER]][PORT_X] &= ~(1 << DisplayCs[CURRENT_DISPLAY].pointer[PIN_PIN]);
        
        // ctrl c ctrl v counter zooi
        int segmentPattern = evilArray[displayDigits[CURRENT_DISPLAY]];
        
        for(int DISPLAY_INDEX = START; DISPLAY_INDEX < maxDisplays; DISPLAY_INDEX++) {
            if(segmentPattern & (1 << DISPLAY_INDEX)) {
                *REG_STORAGE[Displays[DISPLAY_INDEX].pointer[PIN_REGISTER]][PORT_X] |= (1 << Displays[DISPLAY_INDEX].pointer[PIN_PIN]); 
            } else {
                *REG_STORAGE[Displays[DISPLAY_INDEX].pointer[PIN_REGISTER]][PORT_X] &= ~(1 << Displays[DISPLAY_INDEX].pointer[PIN_PIN]); 
            }
            if (CURRENT_DISPLAY == 1) {
            *REG_STORAGE[1][1]|= (1 << PC5);    // DP ON
            } else {
            *REG_STORAGE[1][1] &= ~(1 << PC5);   // DP OFF
            }
        }
        
        delay(5);
    }
}


int main() {
    init();
    Serial.begin(9600); // Initialize serial communication
    millis();
    InitializeIO();
    get_digits(1000);
    // Configure PB5 as input

    // Optional: Add external pull-down resistor to PB5

    while (1) {
        curTime = millis();
        if (vehicle_passed()) {
            Sensors[START].lastActivation = 0;
            currentCount++;
            if (currentCount > 15) {
                currentCount = 0;
            }

            int speed = (60 * 100) / total_time;
            get_digits(speed);
            Serial.println(speed);

        }
        display_counter(currentCount);
        display_speed();
        delay(5);            
    }

    return 0;
}