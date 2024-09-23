#pragma once

#include "pindef.h"

// C
#include <stdint.h>

// Special return values
static const uint16_t kDS18B20_DeviceNotFound = 0xA800;
static const uint16_t kDS18B20_CrcCheckFailed = 0x5000;

/**
 * Trigger all devices on the bus to perform a temperature reading
 * This returns immedidately, but callers must wait for conversion on slaves (max 750ms)
 */
void ds18b20_convert(const gpin_t* io);

/**
 * Read the last temperature conversion from the only probe on the bus
 *
 * If there is a single slave on the one-wire bus the temperature data can be
 * retrieved without scanning for and targeting device addresses.
 *
 * Calling this with more than one slave on the bus will cause data collision.
 */
uint16_t ds18b20_read_single(const gpin_t* io);

/**
 * Read the last temperature conversion from a specific probe
 * Address must be a an array of 8 bytes (uint8_t[8])
 */
uint16_t ds18b20_read_slave(const gpin_t* io, uint8_t* address);