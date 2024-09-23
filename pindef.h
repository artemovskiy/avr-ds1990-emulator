#pragma once

// AVR
#include <avr/io.h>

// C
#include <stdint.h>

/**
 * Generic AVR port pin
 */
typedef struct gpin_t {
    // Pointers to PORT and PIN and DDR registers
    volatile uint8_t *port;
    volatile uint8_t *pin;
    volatile uint8_t *ddr;

    // Bit number in PORT
    uint8_t bit;
} gpin_t;

void gset_input_pullup(const gpin_t* pin);
void gset_input_hiz(const gpin_t* pin);
void gset_output(const gpin_t* pin);
void gset_output_high(const gpin_t* pin);
void gset_output_low(const gpin_t* pin);
void gset_bit(const gpin_t* pin);
void gclear_bit(const gpin_t* pin);
uint8_t gread_bit(const gpin_t* pin);