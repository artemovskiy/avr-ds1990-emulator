#include "ds18b20.h"
#include "crc.h"
#include "onewire.h"

// AVR
#include <util/delay.h>

// Command bytes
static const uint8_t kConvertCommand = 0x44;
static const uint8_t kReadScatchPad = 0xBE;

// Scratch pad data indexes
static const uint8_t kScratchPad_tempLSB = 0;
static const uint8_t kScratchPad_tempMSB = 1;
static const uint8_t kScratchPad_crc = 8;

static uint16_t ds18b20_readScratchPad(const gpin_t* io)
{
    // Read scratchpad into buffer (LSB byte first)
    static const int8_t kScratchPadLength = 9;
    uint8_t buffer[kScratchPadLength];

    for (int8_t i = 0; i < kScratchPadLength; ++i) {
        buffer[i] = onewire_read(io);
    }

    // Check the CRC (9th byte) against the 8 bytes of data
    if (crc8(buffer, 8) != buffer[kScratchPad_crc]) {
        return kDS18B20_CrcCheckFailed;
    }

    // Return the raw 9 to 12-bit temperature value
    return (buffer[kScratchPad_tempMSB] << 8) | buffer[kScratchPad_tempLSB];
}

uint16_t ds18b20_read_single(const gpin_t* io)
{
    // Confirm the device is still alive. Abort if no reply
    if (!onewire_reset(io)) {
        return kDS18B20_DeviceNotFound;
    }

    // Reading a single device, so skip sending a device address
    onewire_skiprom(io);
    onewire_write(io, kReadScatchPad);

    // Read the data from the scratch pad
    return ds18b20_readScratchPad(io);
}

uint16_t ds18b20_read_slave(const gpin_t* io, uint8_t* address)
{
    // Confirm the device is still alive. Abort if no reply
    if (!onewire_reset(io)) {
        return kDS18B20_DeviceNotFound;
    }

    onewire_match_rom(io, address);
    onewire_write(io, kReadScatchPad);

    // Read the data from the scratch pad
    return ds18b20_readScratchPad(io);
}

void ds18b20_convert(const gpin_t* io)
{
    // Send convert command to all devices (this has no response)
    onewire_skiprom(io);
    onewire_write(io, kConvertCommand);
}