#include "onewire.h"
#include "crc.h"

#include <util/delay.h>

void onewire_search_init(onewire_search_state *state)
{
    state->lastZeroBranch = -1;
    state->done = false;

    // Zero-fill the address
    memset(state->address, 0, sizeof(state->address));
}

bool onewire_reset(const gpin_t *io)
{
    // Configure for output
    gset_output_high(io);
    gset_output(io);

    // Pull low for >480uS (master reset pulse)
    gset_output_low(io);
    _delay_us(480);

    // Configure for input
    gset_input_hiz(io);
    _delay_us(70);

    // Look for the line pulled low by a slave
    uint8_t result = gread_bit(io);

    // Wait for the presence pulse to finish
    // This should be less than 240uS, but the master is expected to stay
    // in Rx mode for a minimum of 480uS in total
    _delay_us(460);

    return result == 0;
}

/**
 * Output a Write-0 or Write-1 slot on the One Wire bus
 * A Write-1 slot is generated unless the passed value is zero
 */
static void onewire_write_bit(const gpin_t *io, uint8_t bit)
{
    if (bit != 0)
    { // Write high

        // Pull low for less than 15uS to write a high
        gset_output_low(io);
        _delay_us(5);
        gset_output_high(io);

        // Wait for the rest of the minimum slot time
        _delay_us(55);
    }
    else
    { // Write low

        // Pull low for 60 - 120uS to write a low
        gset_output_low(io);
        _delay_us(55);

        // Stop pulling down line
        gset_output_high(io);

        // Recovery time between slots
        _delay_us(5);
    }
}

// One Wire timing is based on this Maxim application note
// https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
void onewire_write(const gpin_t *io, uint8_t byte)
{
    // Configure for output
    gset_output_high(io);
    gset_output(io);

    for (uint8_t i = 8; i != 0; --i)
    {

        onewire_write_bit(io, byte & 0x1);

        // Next bit (LSB first)
        byte >>= 1;
    }
}

/**
 * Generate a read slot on the One Wire bus and return the bit value
 * Return 0x0 or 0x1
 */
static uint8_t onewire_read_bit(const gpin_t *io)
{
    // Pull the 1-wire bus low for >1uS to generate a read slot
    gset_output_low(io);
    gset_output(io);
    _delay_us(1);

    // Configure for reading (releases the line)
    gset_input_hiz(io);

    // Wait for value to stabilise (bit must be read within 15uS of read slot)
    _delay_us(10);

    uint8_t result = gread_bit(io) != 0;

    // Wait for the end of the read slot
    _delay_us(50);

    return result;
}

uint8_t onewire_read(const gpin_t *io)
{
    uint8_t buffer = 0x0;

    // Configure for input
    gset_input_hiz(io);

    // Read 8 bits (LSB first)
    for (uint8_t bit = 0x01; bit; bit <<= 1)
    {

        // Copy read bit to least significant bit of buffer
        if (onewire_read_bit(io))
        {
            buffer |= bit;
        }
    }

    return buffer;
}

void onewire_match_rom(const gpin_t *io, uint8_t *address)
{
    // Write Match Rom command on bus
    onewire_write(io, 0x55);

    // Send the passed address
    for (uint8_t i = 0; i < 8; ++i)
    {
        onewire_write(io, address[i]);
    }
}

void onewire_skiprom(const gpin_t *io)
{
    onewire_write(io, 0xCC);
}

/**
 * Search procedure for the next ROM addresses
 *
 * This algorithm is bit difficult to understand from the diagrams in Maxim's
 * datasheets and app notes, though its reasonably straight forward once
 * understood.  I've used the name "last zero branch" instead of Maxim's name
 * "last discrepancy", since it describes how this variable is used.
 *
 * A device address has 64 bits. With multiple devices on the bus, some bits
 * are ambiguous.  Each time an ambiguous bit is encountered, a zero is written
 * and the position is marked.  In subsequent searches at ambiguous bits, a one
 * is written at this mark, zeros are written after the mark, and the bit in
 * the previous address is copied before the mark. This effectively steps
 * through all addresses present on the bus.
 *
 * For reference, see either of these documents:
 *
 *  - Maxim application note 187: 1-Wire Search Algorithm
 *    https://www.maximintegrated.com/en/app-notes/index.mvp/id/187
 *
 *  - Maxim application note 937: Book of iButton® Standards (pages 51-54)
 *    https://www.maximintegrated.com/en/app-notes/index.mvp/id/937
 *
 * @see onewire_search()
 * @returns true if a new address was found
 */
static bool _search_next(const gpin_t *io, onewire_search_state *state)
{
    // States of ROM search reads
    enum
    {
        kConflict = 0b00,
        kZero = 0b10,
        kOne = 0b01,
    };

    // Value to write to the current position
    uint8_t bitValue = 0;

    // Keep track of the last zero branch within this search
    // If this value is not updated, the search is complete
    int8_t localLastZeroBranch = -1;

    for (int8_t bitPosition = 0; bitPosition < 64; ++bitPosition)
    {

        // Calculate bitPosition as an index in the address array
        // This is written as-is for readability. Compilers should reduce this to bit shifts and tests
        uint8_t byteIndex = bitPosition / 8;
        uint8_t bitIndex = bitPosition % 8;

        // Configure bus pin for reading
        gset_input_hiz(io);

        // Read the current bit and its complement from the bus
        uint8_t reading = 0;
        reading |= onewire_read_bit(io);      // Bit
        reading |= onewire_read_bit(io) << 1; // Complement of bit (negated)

        switch (reading)
        {
        case kZero:
        case kOne:
            // Bit was the same on all responding devices: it is a known value
            // The first bit is the value we want to write (rather than its complement)
            bitValue = (reading & 0x1);
            break;

        case kConflict:
            // Both 0 and 1 were written to the bus
            // Use the search state to continue walking through devices
            if (bitPosition == state->lastZeroBranch)
            {
                // Current bit is the last position the previous search chose a zero: send one
                bitValue = 1;
            }
            else if (bitPosition < state->lastZeroBranch)
            {
                // Before the lastZeroBranch position, repeat the same choices as the previous search
                bitValue = state->address[byteIndex] & (1 << bitIndex);
            }
            else
            {
                // Current bit is past the lastZeroBranch in the previous search: send zero
                bitValue = 0;
            }

            // Remember the last branch where a zero was written for the next search
            if (bitValue == 0)
            {
                localLastZeroBranch = bitPosition;
            }

            break;

        default:
            // If we see "11" there was a problem on the bus (no devices pulled it low)
            return false;
        }

        // Write bit into address
        if (bitValue == 0)
        {
            state->address[byteIndex] &= ~(1 << bitIndex);
        }
        else
        {
            state->address[byteIndex] |= (bitValue << bitIndex);
        }

        // Configure for output
        gset_output_high(io);
        gset_output(io);

        // Write bit to the bus to continue the search
        onewire_write_bit(io, bitValue);
    }

    // If the no branch points were found, mark the search as done.
    // Otherwise, mark the last zero branch we found for the next search
    if (localLastZeroBranch == -1)
    {
        state->done = true;
    }
    else
    {
        state->lastZeroBranch = localLastZeroBranch;
    }

    // Read a whole address - return success
    return true;
}

static inline bool _search_devices(uint8_t command, const gpin_t *io, onewire_search_state *state)
{
    // Bail out if the previous search was the end
    if (state->done)
    {
        return false;
    }

    if (!onewire_reset(io))
    {
        // No devices present on the bus
        return false;
    }

    onewire_write(io, command);
    return _search_next(io, state);
}

bool onewire_search(const gpin_t *io, onewire_search_state *state)
{
    // Search with "Search ROM" command
    return _search_devices(0xF0, io, state);
}

bool onewire_alarm_search(const gpin_t *io, onewire_search_state *state)
{
    // Search with "Alarm Search" command
    return _search_devices(0xEC, io, state);
}

bool onewire_check_rom_crc(onewire_search_state *state)
{
    // Validate bits 0..56 (bytes 0 - 6) against the CRC in byte 7 (bits 57..63)
    return state->address[7] == crc8(state->address, 7);
}