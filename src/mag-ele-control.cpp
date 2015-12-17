// Pig puppet human
//
// Firmware for attiny13 based controller for the "human" characters in pig
// puppet installation. Might be too artsy and confusing to try to explain the
// whole thing here in an engineer kind of way. I am sorry that the purpose of
// this program might be somewhat obscure. I assure you that it really is.
//
// Created on: 2015-08-14
// Author: Otto Urpelainen
// Email: oturpe@iki.fi

#include "Atmega328pUtils.h"

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#include "MagnetDriver.h"

#ifdef DEBUG
#include "Debug.h"
#endif

/// \brief Turns the indicator led on of off
///
/// \param lit
///    If led is turned on. Otherwise it is turned off.
inline void setIndicator(bool lit) {
    if (lit) {
        PORTB |= BV(PORTB4);
    } else {
        PORTB &= ~BV(PORTB4);
    }
}

int main() {
    #ifdef DEBUG
        /* TODO: implement debugging
        Debug debug(DEBUG_FREQ);
        */
    #endif

    // Set indicator output pin: PB4
    DDRB |= BV(DDB4);

    // Initialize non-inverted pwm in pins OC0A (PD5) and OC0B (PD5) of timer 0
    Atmega328p::initializeTimer0(Atmega328p::PSV_256,
                                 Atmega328p::PWM_FAST,
                                 false);

    DDRD |= BV(DDD6) | BV(DDD5);
    TCCR0A |= BV(COM0A1) | BV(COM0B1);

    // Initialize non-inverted pwm in pins OC2A (PB3) and OC2B (PD3) of timer 2
    Atmega328p::initializeTimer2(Atmega328p::PSV_256,
                                 Atmega328p::PWM_FAST,
                                 false);

    DDRB |= BV(DDB3);
    DDRD |= BV(DDD3);
    TCCR2A |= BV(COM0A1) | BV(COM0B1);

    MagnetDriver driver;

    bool indicatorLit = false;
    uint16_t counter = 0;
    bool morePower = true;
    uint8_t firstValue = 0;
    #define VALUES 50
    uint8_t valueList[VALUES][4] = {
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {0,0,0,0},
                            {32,0,0,0},
                            {64,0,0,0},
                            {96,0,0,0},
                            {128,0,0,0},
                            {160,0,0,0},
                            {192,0,0,0},
                            {224,0,0,0},
                            {255,0,0,20},
                            {224,32,0,40},
                            {192,64,0,60},
                            {160,96,0,80},
                            {128,128,0,100},
                            {96,160,0,120},
                            {64,192,0,140},
                            {32,224,0,160},
                            {0,255,0,180},
                            {0,224,32,200},
                            {0,192,64,220},
                            {0,160,96,240},
                            {0,128,128,0},
                            {0,96,160,20},
                            {0,64,192,40},
                            {0,32,224,60},
                            {0,0,255,80},
                            {32,0,224,100},
                            {64,0,192,120},
                            {96,0,160,140},
                            {128,0,128,160},
                            {160,0,96,180},
                            {192,0,64,200},
                            {224,0,32,240}};
    uint8_t valueIndex = 0;

    while(true) {
        counter += 1;
        _delay_ms(LOOP_DELAY);

        if(counter % INDICATOR_HALF_PERIOD == 0) {
            indicatorLit = !indicatorLit;
            setIndicator(indicatorLit);
        }

        if(counter % MAGNET_RUN_INTERVAL == 0) {
            valueIndex = (valueIndex == VALUES) ? 0 : (valueIndex + 1);
            driver.set(valueList[valueIndex]);
        }
    }
}
