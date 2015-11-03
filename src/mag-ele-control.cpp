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

#include "Attiny45Utils.h"

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#ifdef DEBUG
#include "Debug.h"
#endif

/// \brief Turns the indicator led on of off
///
/// \param lit
///    If led is turned on. Otherwise it is turned off.
inline void setIndicator(bool lit) {
  static uint8_t counter = 1;
  if (lit) {
    PORTB |= BV(PORTB2);
  } else {
    PORTB = ~BV(PORTB2);
  }
  // TEMP: Just testing out the magnet drivers
  counter += 16;
  OCR0A = counter;
  OCR0B = 0xff -counter;
  OCR1B = counter;
}

/*
// How many more sensors until presence of pig is detected
uint8_t pigDetectionDelay = LDR_DELAY;

/// \brief
///    Tracks if light dependent resistors have had different exposure LDR_DELAY
///    times consecutively. This is interpreted to mean that pig has appeared in
///    front of human.
///
/// \return
///    If pig is detected
inline bool pollSensors() {
    if((PINB & BV(PINB3)) ^ (PINB & BV(PINB4))) {
        if(pigDetectionDelay > 0) {
            pigDetectionDelay--;
        }
    } else {
        pigDetectionDelay = LDR_DELAY;
    }

    return pigDetectionDelay == 0;
}

uint16_t humanInactivityDelay = 0;

inline void runHuman() {
    if(humanInactivityDelay > 0) {
        humanInactivityDelay--;
        PORTB |= BV(PORTB1) | BV(PORTB2);
        OCR0B = 0xff - HUMAN_MOTOR_DUTY_CYCLE;
        return;
    }

    PORTB &= ~BV(PORTB1) & ~BV(PORTB2);
    OCR0B = 0xff;
}
*/

int main() {
    #ifdef DEBUG
        /* TODO: implement debugging
        Debug debug(DEBUG_FREQ);
        */
    #endif

    // Set indicator output pin: PB2
        DDRB |= BV(DDB2);

    // Set magnet output pins: PB0, PB1, PB4
    DDRB |= BV(DDB0) | BV(DDB1) | BV(DDB4);

    // Initialize non-inverted pwm in pins OC0A (PB0) and OC0B (PB1) of timer 0
    TCCR0A |= BV(WGM01) | BV(WGM00);
    TCCR0A |= BV(COM0A1) | BV(COM0A0);
    TCCR0A |= BV(COM0B1) | BV(COM0B0);
    Attiny45::setTimer0Prescaler(Attiny45::PSV_64);
    // Initialize non-invertedpwm in pin  OC1B (PB4) of timer 1
    GTCCR |= BV(PWM1B);
    GTCCR |= BV(COM1B1);
    OCR1C = 0xff;
    Attiny45::setTimer1Prescaler(Attiny45::PSV_64);

    bool indicatorLit = false;
    uint16_t counter = 0;
    while(true) {
        counter += 1;
        _delay_ms(LOOP_DELAY);

        if(counter % INDICATOR_HALF_PERIOD == 0) {
            indicatorLit = !indicatorLit;
            setIndicator(indicatorLit);
        }
    }
}
