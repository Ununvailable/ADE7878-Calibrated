// led_flash_all_pins.cpp
// Flash ALL GPIO pins - 1 second ON, 1 second OFF

#include <Arduino.h>

// All GPIO pins available on STM32F429ZIT6 (LQFP144)
// Total: 140 GPIO pins across ports A-K
const int gpioPins[] = {
    // // Port A (PA0-PA15)
    // PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    // PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,

    // // Port B (PB0-PB15)
    // PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
    // PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,

    // // Port C (PC0-PC15)
    // PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7,
    // PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,

    // // Port D (PD0-PD15)
    // PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7,
    // PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,

    // // Port E (PE0-PE15)
    // PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7,
    // PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,

    // // Port F (PF0-PF15)
    // PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7,
    // PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,

    // // Port G (PG0-PG15)
    // PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7,
    // PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
  
    // // Port H (PH0-PH15)
    // PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7,
    // PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15,

    // // Port I (PI0-PI15)
    // PI0, PI1, PI2, PI3, PI4, PI5, PI6, PI7,
    // PI8, PI9, PI10, PI11, PI12, PI13, PI14, PI15,

    // // Port J (PJ0-PJ15)
    // PJ0, PJ1, PJ2, PJ3, PJ4, PJ5, PJ6, PJ7,
    // PJ8, PJ9, PJ10, PJ11, PJ12, PJ13, PJ14, PJ15,

    // // Port K (PK0-PK7)
    // PK0, PK1, PK2, PK3, PK4, PK5, PK6, PK7
};

const int numPins = sizeof(gpioPins) / sizeof(gpioPins[0]);

HardwareSerial Serial8(PE0, PE1);

void setup() {
    Serial8.begin(9600);
    // Initialize all GPIO pins as outputs
    for (int i = 0; i < numPins; i++) {
        pinMode(gpioPins[i], OUTPUT);
        digitalWrite(gpioPins[i], LOW);  // Start with all pins LOW
    }
}

void loop() {
    // Turn all pins HIGH
    // for (int i = 0; i < numPins; i++) {
    //     digitalWrite(gpioPins[i], HIGH);
    // }
    // delay(1000);  // Wait 1 second
    
    // // Turn all pins LOW
    // for (int i = 0; i < numPins; i++) {
    //     digitalWrite(gpioPins[i], LOW);
    // }
    delay(2000);  // Wait 1 second
    Serial8.println(F("Yo"));
}