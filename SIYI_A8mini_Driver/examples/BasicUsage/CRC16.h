/**
 * @file CRC16.h
 * @brief CRC16 implementation exactly as specified in the SIYI Gimbal Camera
 *        External SDK Protocol Document (Chapter 4 - CRC16 Checksum).
 *
 * Polynomial: G(X) = X^16 + X^12 + X^5 + 1  (CRC-CCITT), init = 0x0000.
 *
 * Verified against the protocol document's own worked example:
 *   Heartbeat packet: 55 66 01 01 00 00 00 00 00 59 8B
 *   CRC16 over bytes [55 66 01 01 00 00 00 00 00] = 0x8B59
 *   (transmitted low byte first -> 59 8B), which matches exactly.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace siyi {

class CRC16 {
public:
    /**
     * @brief Calculate CRC16 over a buffer, exactly matching the protocol's
     *        CRC16_cal() reference implementation.
     * @param data Pointer to the data to checksum.
     * @param len  Number of bytes to checksum.
     * @param crcInit Initial CRC value (protocol always uses 0).
     * @return 16-bit CRC value.
     */
    static uint16_t calculate(const uint8_t *data, size_t len, uint16_t crcInit = 0);

private:
    static const uint16_t kTable[256];
};

} // namespace siyi
