#pragma once

#include "pindef.h"

// C
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/**
 * State for the onewire_search function
 * This must be initialised with onewire_search_init() before use.
 */
typedef struct onewire_search_state {

    // The highest bit position where a bit was ambiguous and a zero was written
    int8_t lastZeroBranch;

    // Internal flag to indicate if the search is complete
    // This flag is set once there are no more branches to search
    bool done;

    // Discovered 64-bit device address (LSB first)
    // After a successful search, this contains the found device address.
    // During a search this is overwritten LSB-first with a new address.
    uint8_t address[8];

} onewire_search_state;

/**
 * Send a reset pulse
 *
 * Returns true if One Wire devices were detected on the bus
 */
bool onewire_reset(const gpin_t* io);

/**
 * Write a byte as described in Maxim's One Wire protocol
 */
void onewire_write(const gpin_t* io, uint8_t byte);

/**
 * Read a byte as described in Maxim's One Wire protocol
 */
uint8_t onewire_read(const gpin_t* io);

/**
 * Skip sending a device address
 */
void onewire_skiprom(const gpin_t* io);

/**
 * Address a specific device
 */
void onewire_match_rom(const gpin_t* io, uint8_t* address);

/**
 * Reset a search state for use in a search
 */
 void onewire_search_init(onewire_search_state* state);

/**
 * Look for the next slave address on the bus
 *
 * Before the first search call, the state parameter must be initialised using
 * onewire_init_search(state). The same state must be passed to subsequent calls
 * to discover all available devices.
 *
 * The caller is responsible for performing a CRC check on the result if desired.
 *
 * @returns true if a new address was found
 */
bool onewire_search(const gpin_t* io, onewire_search_state* state);

/**
 * Look for the next slave address on the bus with an alarm condition
 * @see onewire_search()
 */
bool onewire_alarm_search(const gpin_t* io, onewire_search_state* state);

/**
 * Return true if the CRC byte in a ROM address validates
 */
bool onewire_check_rom_crc(onewire_search_state* state);