#include "MagnetDriver.h"

#include <avr/io.h>

MagnetDriver::MagnetDriver() {
}

MagnetDriver::~MagnetDriver() {
}

void MagnetDriver::set(uint8_t * values) {
    OCR0A = values[0];
    OCR0B = values[1];
    OCR2A = values[2];
}