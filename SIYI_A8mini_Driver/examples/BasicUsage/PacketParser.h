/**
 * @file PacketParser.h
 * @brief Byte-at-a-time state machine parser for SIYI protocol frames.
 *
 * Design goals (see driver README "Lessons applied"):
 *  - Never assumes a full frame arrives in one UART read.
 *  - Supports fragmented frames (fed one byte, or many bytes, at a time).
 *  - Supports back-to-back frames in a single buffer (call feed()
 *    repeatedly; each call consumes exactly one byte and may or may not
 *    complete a packet).
 *  - Discards corrupted frames safely: on CRC failure, or on a stray
 *    byte that can't be part of a valid frame, the parser resyncs by
 *    dropping bytes until it finds a fresh STX rather than getting stuck.
 *  - All buffer sizes are configurable via constructor / template
 *    capacity, no magic numbers.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "Protocol.h"
#include "CRC16.h"

namespace siyi {

/**
 * @brief Result of a fully parsed and CRC-validated packet.
 *
 * IMPORTANT: `data` points into the PacketParser's own internal payload
 * buffer, which is overwritten on the parser's next call. Consume
 * `data`/`dataLen` synchronously (copy out what you need) before feeding
 * the parser again -- do not retain a ParsedPacket (or its `data`
 * pointer) across multiple feed() calls. SIYI.h follows this rule: it
 * copies payload bytes out in handlePacket() immediately.
 */
struct ParsedPacket {
    uint8_t ctrl = 0;
    uint16_t seq = 0;
    uint8_t cmdId = 0;
    const uint8_t *data = nullptr; ///< points into parser's internal buffer
    uint16_t dataLen = 0;
};

/**
 * @brief Streaming, fragmentation-tolerant packet parser.
 *
 * Usage:
 *   PacketParser<256> parser; // 256-byte max payload
 *   while (serial.available()) {
 *       ParsedPacket pkt;
 *       if (parser.feed(serial.read(), pkt)) {
 *           // pkt is valid, CRC-checked, ready to use
 *       }
 *   }
 *
 * @tparam MaxPayload Maximum DATA field size this parser will accept.
 *         Frames declaring a larger Data_len are treated as corrupt and
 *         discarded (parser resyncs on the next STX).
 */
template <size_t MaxPayload = 256>
class PacketParser {
public:
    PacketParser() { reset(); }

    /**
     * @brief Feed a single received byte into the parser.
     * @param byte Incoming byte from UART.
     * @param outPacket Filled in if this byte completes a valid packet.
     * @return true if outPacket now holds a valid, CRC-checked packet.
     */
    bool feed(uint8_t byte, ParsedPacket &outPacket) {
        switch (state_) {
            case State::WaitStx0:
                if (byte == kStx0) {
                    beginFrame(byte);
                    state_ = State::WaitStx1;
                }
                // else: stay in WaitStx0, discard stray byte.
                return false;

            case State::WaitStx1:
                if (byte == kStx1) {
                    appendHeaderByte(byte);
                    state_ = State::Ctrl;
                } else if (byte == kStx0) {
                    // Resync: this byte could be a fresh STX0.
                    beginFrame(byte);
                    // stay in WaitStx1
                } else {
                    reset();
                }
                return false;

            case State::Ctrl:
                appendHeaderByte(byte);
                ctrl_ = byte;
                state_ = State::LenLow;
                return false;

            case State::LenLow:
                appendHeaderByte(byte);
                dataLen_ = byte;
                state_ = State::LenHigh;
                return false;

            case State::LenHigh:
                appendHeaderByte(byte);
                dataLen_ = static_cast<uint16_t>(dataLen_ | (static_cast<uint16_t>(byte) << 8));
                if (dataLen_ > MaxPayload) {
                    // Corrupt or oversized frame for this buffer; resync.
                    reset();
                    return false;
                }
                state_ = State::SeqLow;
                return false;

            case State::SeqLow:
                appendHeaderByte(byte);
                seq_ = byte;
                state_ = State::SeqHigh;
                return false;

            case State::SeqHigh:
                appendHeaderByte(byte);
                seq_ = static_cast<uint16_t>(seq_ | (static_cast<uint16_t>(byte) << 8));
                state_ = State::CmdId;
                return false;

            case State::CmdId:
                appendHeaderByte(byte);
                cmdId_ = byte;
                dataReceived_ = 0;
                state_ = (dataLen_ > 0) ? State::Data : State::CrcLow;
                return false;

            case State::Data:
                payload_[dataReceived_++] = byte;
                appendCrcInputByte(byte);
                if (dataReceived_ >= dataLen_) {
                    state_ = State::CrcLow;
                }
                return false;

            case State::CrcLow:
                crcRx_ = byte;
                state_ = State::CrcHigh;
                return false;

            case State::CrcHigh: {
                crcRx_ = static_cast<uint16_t>(crcRx_ | (static_cast<uint16_t>(byte) << 8));
                bool ok = finalizeFrame(outPacket);
                reset();
                return ok;
            }
        }
        return false; // unreachable
    }

    /// Discards any partially-received frame and returns to the idle state.
    void reset() {
        state_ = State::WaitStx0;
        headerLen_ = 0;
        dataLen_ = 0;
        dataReceived_ = 0;
        crcInputLen_ = 0;
    }

private:
    enum class State {
        WaitStx0, WaitStx1, Ctrl, LenLow, LenHigh,
        SeqLow, SeqHigh, CmdId, Data, CrcLow, CrcHigh
    };

    void beginFrame(uint8_t firstByte) {
        headerLen_ = 0;
        crcInputLen_ = 0;
        crcBuf_[crcInputLen_++] = firstByte;
        headerBuf_[headerLen_++] = firstByte;
    }

    void appendHeaderByte(uint8_t b) {
        headerBuf_[headerLen_++] = b;
        appendCrcInputByte(b);
    }

    void appendCrcInputByte(uint8_t b) {
        if (crcInputLen_ < sizeof(crcBuf_)) {
            crcBuf_[crcInputLen_++] = b;
        }
        // If this ever overflows (payload > MaxPayload), the LenHigh
        // handler above already rejected the frame before we get here.
    }

    bool finalizeFrame(ParsedPacket &outPacket) {
        uint16_t computed = CRC16::calculate(crcBuf_, crcInputLen_);
        if (computed != crcRx_) {
            return false; // corrupted frame, silently discarded
        }
        outPacket.ctrl = ctrl_;
        outPacket.seq = seq_;
        outPacket.cmdId = cmdId_;
        outPacket.data = payload_;
        outPacket.dataLen = dataLen_;
        return true;
    }

    State state_ = State::WaitStx0;

    // Header bytes (STX..CMD_ID), kept for completeness/debug; CRC input
    // buffer below is what's actually used for validation.
    uint8_t headerBuf_[kHeaderLen] = {0};
    size_t headerLen_ = 0;

    uint8_t ctrl_ = 0;
    uint16_t dataLen_ = 0;
    uint16_t seq_ = 0;
    uint8_t cmdId_ = 0;

    uint8_t payload_[MaxPayload] = {0};
    uint16_t dataReceived_ = 0;

    uint16_t crcRx_ = 0;

    // Everything from STX0 through the last DATA byte, used as CRC input.
    uint8_t crcBuf_[kHeaderLen + MaxPayload] = {0};
    size_t crcInputLen_ = 0;
};

} // namespace siyi
