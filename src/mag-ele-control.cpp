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
    #define VALUES 25
    uint8_t valueList[VALUES][3] = {
                            {255,0,0},
                            {224,32,0},
                            {192,64,0},
                            {160,96,0},
                            {128,128,0},
                            {96,160,0},
                            {64,192,0},
                            {32,224,0},
                            {0,255,0},
                            {0,224,32},
                            {0,192,64},
                            {0,160,96},
                            {0,128,128},
                            {0,96,160},
                            {0,64,192},
                            {0,32,224},
                            {0,0,255},
                            {32,0,224},
                            {64,0,192},
                            {96,0,160},
                            {128,0,128},
                            {160,0,96},
                            {192,0,64},
                            {224,0,32}};
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
