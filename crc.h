// C
#include <stdint.h>

/**
 * Calculate the CRC8 for an array of bytes
 *
 * This uses the polynomial (x^8 + x^5 + x^4 + 1)
 *
 * @returns the computed CRC8
 */
uint8_t crc8(uint8_t* data, uint8_t len);