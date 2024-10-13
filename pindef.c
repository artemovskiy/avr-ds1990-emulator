#include "pindef.h"

void gset_input_pullup(const gpin_t* pin) {
    *(pin->ddr) &= ~_BV(pin->bit);
    gset_output_high(pin);
}

void gset_input_hiz(const gpin_t* pin) {
    *(pin->ddr) &= ~_BV(pin->bit);
    gset_output_low(pin);
}

void gset_output(const gpin_t* pin) {
    *(pin->ddr) |= _BV(pin->bit);
}

void gset_output_high(const gpin_t* pin) {
    *(pin->port) |= _BV(pin->bit);
}

void gset_output_low(const gpin_t* pin) {
    *(pin->port) &= ~_BV(pin->bit);
}

void gset_bit(const gpin_t* pin) {
    *(pin->port) |= _BV(pin->bit);
}

void gclear_bit(const gpin_t* pin) {
    *(pin->port) &= ~_BV(pin->bit);
}

uint8_t gread_bit(const gpin_t* pin) {
    return *(pin->pin) & _BV(pin->bit);
}