/**
 * @file SIYI.h
 * @brief ESP32 / Arduino driver for the SIYI A8 Mini Gimbal Camera,
 *        implementing the commands in the "SIYI Gimbal Camera External SDK
 *        Protocol Document" (V0.1.1) relevant to gimbal & camera control.
 *
 * Target: ESP32, Arduino framework, PlatformIO or Arduino IDE.
 * UART: HardwareSerial only (SoftwareSerial is not supported by design --
 *       it cannot reliably sustain 115200 baud on the ESP32).
 *
 * Scope note: the protocol document also defines thermal-imaging, laser
 * rangefinder, GPS/RC telemetry-injection and video-stitching commands
 * for other SIYI cameras (ZR10/ZR30/ZT30/quad-spectrum). Those are
 * outside the A8 Mini's feature set and outside the requested API
 * surface, so they are not implemented here. Every command that IS
 * implemented is listed in Protocol.h's CommandId enum.
 *
 * Threading / blocking model:
 *  - update() must be called frequently (e.g. every loop() iteration).
 *    It drains available UART bytes into the packet parser and, for any
 *    fully-parsed valid packet, either fulfills a pending blocking
 *    request or forwards the packet to the optional user callback.
 *  - "get/set with ack" methods block internally, but only up to an
 *    explicit, caller-supplied timeout (default a few hundred ms) -- they
 *    never block forever. Internally they simply pump update() in a
 *    loop, so the parser keeps making progress even while "blocked".
 *  - Commands the protocol defines as having NO ACK (see Protocol.h)
 *    are fire-and-forget: the driver does not wait for a response that
 *    will never come.
 */
#pragma once

#include <Arduino.h>
#include "Protocol.h"
#include "PacketParser.h"
#include "CRC16.h"

namespace siyi {

/// Optional callback for the async, unsolicited 0x0B FunctionFeedback push.
using FunctionFeedbackCallback = void (*)(FunctionFeedbackInfo info);

/**
 * @tparam RxPayloadCapacity Max payload bytes the internal parser will
 *         accept. All implemented ACK payloads are well under 64 bytes;
 *         the default leaves headroom without wasting much RAM.
 */
template <size_t RxPayloadCapacity = 64>
class SIYIDriver {
public:
    explicit SIYIDriver(HardwareSerial &serial) : serial_(serial) {}

    /**
     * @brief Initialize the driver. Does NOT call serial.begin() -- the
     *        caller owns UART pin configuration (see README), matching
     *        the requirement to construct HardwareSerial externally.
     */
    void begin() {
        parser_.reset();
        seqCounter_ = 0;
        pending_.active = false;
    }

    /**
     * @brief Pump the driver: read available UART bytes, parse frames,
     *        fulfill pending blocking requests, dispatch async callbacks.
     *        Call this every loop() iteration (and internally while a
     *        blocking call is waiting).
     */
    void update() {
        while (serial_.available() > 0) {
            uint8_t b = static_cast<uint8_t>(serial_.read());
            ParsedPacket pkt;
            if (parser_.feed(b, pkt)) {
                handlePacket(pkt);
            }
        }
        if (heartbeatIntervalMs_ > 0 && millis() - lastHeartbeatMs_ >= heartbeatIntervalMs_) {
            sendHeartbeat();
            lastHeartbeatMs_ = millis();
        }
    }

    /// Enable/disable verbose TX/RX debug logging on an Arduino Print stream.
    void setDebug(bool enable, Print &out = Serial) {
        debugEnabled_ = enable;
        debugOut_ = &out;
    }

    /// Optional: auto-send heartbeat (CMD 0x00) every intervalMs from update(). 0 = disabled.
    void setHeartbeatInterval(uint32_t intervalMs) {
        heartbeatIntervalMs_ = intervalMs;
        lastHeartbeatMs_ = millis();
    }

    /// Register a callback for unsolicited 0x0B FunctionFeedback events.
    void onFunctionFeedback(FunctionFeedbackCallback cb) { feedbackCb_ = cb; }

    // ------------------------------------------------------------------
    // Read (query) commands
    // ------------------------------------------------------------------

    bool getFirmwareVersion(FirmwareVersionAck &out, uint32_t timeoutMs = 300) {
        return requestAck(CommandId::FirmwareVersion, nullptr, 0, &out, sizeof(out), timeoutMs);
    }

    bool getHardwareId(HardwareIdAck &out, uint32_t timeoutMs = 300) {
        return requestAck(CommandId::HardwareId, nullptr, 0, &out, sizeof(out), timeoutMs);
    }

    /// "Gimbal status" -- record state, HDR, motion mode, mounting dir, etc.
    bool getGimbalStatus(CameraSystemInfoAck &out, uint32_t timeoutMs = 300) {
        return requestAck(CommandId::CameraSystemInfo, nullptr, 0, &out, sizeof(out), timeoutMs);
    }

    bool getGimbalAttitude(GimbalAttitudeAck &out, uint32_t timeoutMs = 300) {
        return requestAck(CommandId::GimbalAttitude, nullptr, 0, &out, sizeof(out), timeoutMs);
    }

    // ------------------------------------------------------------------
    // Gimbal motion
    // ------------------------------------------------------------------

    /// One-key centering (blocking, has ACK). Returns success flag via sta.
    bool center(CenterPos pos, uint8_t &sta, uint32_t timeoutMs = 300) {
        CenterGimbalSend req{static_cast<uint8_t>(pos)};
        CenterGimbalAck ack{};
        bool ok = requestAck(CommandId::CenterGimbal, &req, sizeof(req), &ack, sizeof(ack), timeoutMs);
        sta = ack.sta;
        return ok;
    }
    /// Convenience overload: default one-key center, ignore result detail.
    bool center(uint32_t timeoutMs = 300) {
        uint8_t sta;
        return center(CenterPos::OneKeyCenter, sta, timeoutMs);
    }

    /**
     * @brief Manual rotation (joystick-style continuous control).
     * Fire-and-forget variant: intended to be called at high rate while a
     * stick/touch is held. Ignoring the per-frame ACK here is intentional
     * -- waiting on every frame would throttle control responsiveness.
     */
    void rotate(int8_t yawSpeed, int8_t pitchSpeed) {
        GimbalRotationSend req{yawSpeed, pitchSpeed};
        sendCommand(CommandId::GimbalRotation, &req, sizeof(req));
    }
    /// Blocking variant, if you need the ACK's success flag.
    bool rotateBlocking(int8_t yawSpeed, int8_t pitchSpeed, uint8_t &sta, uint32_t timeoutMs = 300) {
        GimbalRotationSend req{yawSpeed, pitchSpeed};
        GimbalRotationAck ack{};
        bool ok = requestAck(CommandId::GimbalRotation, &req, sizeof(req), &ack, sizeof(ack), timeoutMs);
        sta = ack.sta;
        return ok;
    }
    /// Convenience: stop rotation (send 0,0).
    void stop() { rotate(0, 0); }

    /**
     * @brief Absolute angle control. Clamps to the A8 Mini's documented
     *        range (yaw +/-135.0 deg, pitch -90.0..25.0 deg) before send.
     */
    bool setAngle(float yawDeg, float pitchDeg, SetGimbalAttitudeAck &out, uint32_t timeoutMs = 300) {
        yawDeg = clampf(yawDeg, kA8MiniYawMinDeg, kA8MiniYawMaxDeg);
        pitchDeg = clampf(pitchDeg, kA8MiniPitchMinDeg, kA8MiniPitchMaxDeg);
        SetGimbalAttitudeSend req{};
        req.yaw = static_cast<int16_t>(yawDeg * 10.0f);
        req.pitch = static_cast<int16_t>(pitchDeg * 10.0f);
        return requestAck(CommandId::SetGimbalAttitude, &req, sizeof(req), &out, sizeof(out), timeoutMs);
    }
    bool setAngle(float yawDeg, float pitchDeg, uint32_t timeoutMs = 300) {
        SetGimbalAttitudeAck out{};
        return setAngle(yawDeg, pitchDeg, out, timeoutMs);
    }

    // ------------------------------------------------------------------
    // Zoom / focus
    // ------------------------------------------------------------------

    bool zoomIn(uint16_t &zoomMultiple, uint32_t timeoutMs = 300) { return manualZoom(1, zoomMultiple, timeoutMs); }
    bool zoomOut(uint16_t &zoomMultiple, uint32_t timeoutMs = 300) { return manualZoom(-1, zoomMultiple, timeoutMs); }
    bool zoomStop(uint16_t &zoomMultiple, uint32_t timeoutMs = 300) { return manualZoom(0, zoomMultiple, timeoutMs); }
    /// Fire-and-forget variants (no wait on ACK), for continuous button-hold use.
    void zoomIn()   { ManualZoomSend req{1};  sendCommand(CommandId::ManualZoom, &req, sizeof(req)); }
    void zoomOut()  { ManualZoomSend req{-1}; sendCommand(CommandId::ManualZoom, &req, sizeof(req)); }
    void zoomStop() { ManualZoomSend req{0};  sendCommand(CommandId::ManualZoom, &req, sizeof(req)); }

    /// Absolute zoom, e.g. absoluteZoom(6.5f) -> 6.5x. Range 1.0 .. 30.0 (0x1E).
    bool absoluteZoom(float zoomLevel, uint8_t &ackOk, uint32_t timeoutMs = 300) {
        zoomLevel = clampf(zoomLevel, 1.0f, 30.0f);
        AbsoluteZoomSend req{};
        req.absolute_movement_int = static_cast<uint8_t>(zoomLevel);
        req.absolute_movement_float = static_cast<uint8_t>((zoomLevel - static_cast<int>(zoomLevel)) * 10.0f + 0.5f);
        AbsoluteZoomAck ack{};
        bool ok = requestAck(CommandId::AbsoluteZoom, &req, sizeof(req), &ack, sizeof(ack), timeoutMs);
        ackOk = ack.absolute_movement_ask;
        return ok;
    }

    bool focusFar(uint8_t &sta, uint32_t timeoutMs = 300) { return manualFocus(1, sta, timeoutMs); }
    bool focusNear(uint8_t &sta, uint32_t timeoutMs = 300) { return manualFocus(-1, sta, timeoutMs); }
    bool focusStop(uint8_t &sta, uint32_t timeoutMs = 300) { return manualFocus(0, sta, timeoutMs); }
    void focusFar()  { ManualFocusSend req{1};  sendCommand(CommandId::ManualFocus, &req, sizeof(req)); }
    void focusNear() { ManualFocusSend req{-1}; sendCommand(CommandId::ManualFocus, &req, sizeof(req)); }
    void focusStop() { ManualFocusSend req{0};  sendCommand(CommandId::ManualFocus, &req, sizeof(req)); }

    /// Only supported by optical-zoom cameras (per spec note on 0x04).
    bool autoFocus(uint16_t touchX, uint16_t touchY, uint8_t &sta, uint32_t timeoutMs = 300) {
        AutoFocusSend req{1, touchX, touchY};
        AutoFocusAck ack{};
        bool ok = requestAck(CommandId::AutoFocus, &req, sizeof(req), &ack, sizeof(ack), timeoutMs);
        sta = ack.sta;
        return ok;
    }

    // ------------------------------------------------------------------
    // Photo / recording / motion mode / video output (CMD 0x0C, NO ACK
    // per protocol -- these are fire-and-forget by design; observe
    // results via onFunctionFeedback()).
    // ------------------------------------------------------------------

    void takePhoto() { sendFuncType(FuncType::CapturePhoto); }

    /**
     * @brief Toggle recording start/stop.
     * NOTE (documented ambiguity, not guessed silently): the protocol
     * only defines FuncType 2 as "Start recording" -- it does not define
     * a separate "stop recording" code. Field behavior on other SIYI
     * cameras is that this code toggles recording state, so
     * startRecording() and stopRecording() both send the same FuncType 2
     * and rely on the toggle behavior; check record_sta via
     * getGimbalStatus() to confirm the resulting state.
     */
    void startRecording() { sendFuncType(FuncType::StartRecording); }
    void stopRecording() { sendFuncType(FuncType::StartRecording); }
    void toggleRecording() { sendFuncType(FuncType::StartRecording); }

    void setMotionMode(GimbalMotionMode mode) {
        switch (mode) {
            case GimbalMotionMode::Lock:   sendFuncType(FuncType::LockMode); break;
            case GimbalMotionMode::Follow: sendFuncType(FuncType::FollowMode); break;
            case GimbalMotionMode::FPV:    sendFuncType(FuncType::FpvMode); break;
        }
    }

    /// Reboot required for these three (per spec).
    void enableHdmiOutput() { sendFuncType(FuncType::EnableHdmi); }
    void enableCvbsOutput() { sendFuncType(FuncType::EnableCvbs); }
    void disableVideoOutput() { sendFuncType(FuncType::DisableHdmiCvbs); }

    void tiltDownward() { sendFuncType(FuncType::TiltDownward); }
    void toggleZoomLinkage() { sendFuncType(FuncType::ZoomLinkage); }

    // ------------------------------------------------------------------
    // Heartbeat (CMD 0x00, no ACK, TCP-connection use per spec; harmless
    // to send over UART as a liveness ping if desired)
    // ------------------------------------------------------------------
    void sendHeartbeat() { sendCommand(CommandId::Heartbeat, nullptr, 0); }

private:
    // ------------------------------------------------------------------
    // Internal: framing / TX
    // ------------------------------------------------------------------

    uint16_t sendCommand(CommandId cmd, const void *payload, uint16_t payloadLen) {
        uint16_t seq = nextSeq();
        uint8_t frame[kHeaderLen + RxPayloadCapacity + kCrcLen];
        size_t idx = 0;
        frame[idx++] = kStx0;
        frame[idx++] = kStx1;
        frame[idx++] = kCtrlByte;
        frame[idx++] = static_cast<uint8_t>(payloadLen & 0xFF);
        frame[idx++] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
        frame[idx++] = static_cast<uint8_t>(seq & 0xFF);
        frame[idx++] = static_cast<uint8_t>((seq >> 8) & 0xFF);
        frame[idx++] = static_cast<uint8_t>(cmd);
        if (payload != nullptr && payloadLen > 0) {
            memcpy(&frame[idx], payload, payloadLen);
            idx += payloadLen;
        }
        uint16_t crc = CRC16::calculate(frame, idx);
        frame[idx++] = static_cast<uint8_t>(crc & 0xFF);
        frame[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

        serial_.write(frame, idx);
        debugTx(cmd, seq, payload ? static_cast<const uint8_t *>(payload) : nullptr, payloadLen);
        return seq;
    }

    uint16_t nextSeq() {
        uint16_t s = seqCounter_;
        seqCounter_ = static_cast<uint16_t>(seqCounter_ + 1); // wraps at 65535 naturally
        return s;
    }

    // ------------------------------------------------------------------
    // Internal: RX dispatch
    // ------------------------------------------------------------------

    void handlePacket(const ParsedPacket &pkt) {
        debugRx(pkt);

        // Unsolicited push: 0x0B FunctionFeedback.
        if (pkt.cmdId == static_cast<uint8_t>(CommandId::FunctionFeedback)) {
            if (feedbackCb_ != nullptr && pkt.dataLen >= sizeof(FunctionFeedbackAck)) {
                auto *fb = reinterpret_cast<const FunctionFeedbackAck *>(pkt.data);
                feedbackCb_(static_cast<FunctionFeedbackInfo>(fb->info_type));
            }
            return;
        }

        // Fulfill a pending blocking request if this is its matching ACK.
        if (pending_.active && pkt.cmdId == pending_.cmdId && pkt.seq == pending_.seq) {
            size_t copyLen = pkt.dataLen < pending_.outLen ? pkt.dataLen : pending_.outLen;
            if (pending_.out != nullptr && copyLen > 0) {
                memcpy(pending_.out, pkt.data, copyLen);
            }
            pending_.fulfilled = true;
            pending_.active = false;
        }
    }

    /**
     * @brief Send a command and block (bounded by timeoutMs) pumping
     *        update() until the matching ACK (by CMD_ID + SEQ) arrives.
     * @return true if the ACK arrived before the timeout.
     */
    bool requestAck(CommandId cmd, const void *reqPayload, uint16_t reqLen,
                     void *outAck, size_t outAckLen, uint32_t timeoutMs) {
        uint16_t seq = sendCommand(cmd, reqPayload, reqLen);

        pending_.active = true;
        pending_.fulfilled = false;
        pending_.cmdId = static_cast<uint8_t>(cmd);
        pending_.seq = seq;
        pending_.out = outAck;
        pending_.outLen = outAckLen;

        uint32_t start = millis();
        while (!pending_.fulfilled) {
            update();
            if (pending_.fulfilled) break;
            if (millis() - start >= timeoutMs) {
                pending_.active = false;
                debugTimeout(cmd, seq);
                return false;
            }
        }
        return true;
    }

    bool manualZoom(int8_t dir, uint16_t &zoomMultiple, uint32_t timeoutMs) {
        ManualZoomSend req{dir};
        ManualZoomAck ack{};
        bool ok = requestAck(CommandId::ManualZoom, &req, sizeof(req), &ack, sizeof(ack), timeoutMs);
        zoomMultiple = ack.zoom_multiple;
        return ok;
    }

    bool manualFocus(int8_t dir, uint8_t &sta, uint32_t timeoutMs) {
        ManualFocusSend req{dir};
        ManualFocusAck ack{};
        bool ok = requestAck(CommandId::ManualFocus, &req, sizeof(req), &ack, sizeof(ack), timeoutMs);
        sta = ack.sta;
        return ok;
    }

    void sendFuncType(FuncType f) {
        PhotoVideoControlSend req{static_cast<uint8_t>(f)};
        sendCommand(CommandId::PhotoVideoControl, &req, sizeof(req));
    }

    static float clampf(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    // ------------------------------------------------------------------
    // Debug helpers (compiled in always, gated by debugEnabled_ flag so
    // release-mode overhead is just one branch, no #ifdef divergence).
    // ------------------------------------------------------------------
    void debugTx(CommandId cmd, uint16_t seq, const uint8_t *payload, uint16_t len) {
        if (!debugEnabled_ || debugOut_ == nullptr) return;
        debugOut_->printf("[SIYI][TX] cmd=0x%02X seq=%u len=%u data=", static_cast<unsigned>(cmd), seq, len);
        printHex(payload, len);
        debugOut_->println();
    }

    void debugRx(const ParsedPacket &pkt) {
        if (!debugEnabled_ || debugOut_ == nullptr) return;
        debugOut_->printf("[SIYI][RX] cmd=0x%02X seq=%u ctrl=0x%02X len=%u data=",
                           pkt.cmdId, pkt.seq, pkt.ctrl, pkt.dataLen);
        printHex(pkt.data, pkt.dataLen);
        debugOut_->println();
    }

    void debugTimeout(CommandId cmd, uint16_t seq) {
        if (!debugEnabled_ || debugOut_ == nullptr) return;
        debugOut_->printf("[SIYI][TIMEOUT] cmd=0x%02X seq=%u\n", static_cast<unsigned>(cmd), seq);
    }

    void printHex(const uint8_t *buf, uint16_t len) {
        if (debugOut_ == nullptr) return;
        for (uint16_t i = 0; i < len; ++i) {
            if (buf != nullptr) debugOut_->printf("%02X ", buf[i]);
        }
    }

    HardwareSerial &serial_;
    PacketParser<RxPayloadCapacity> parser_;
    uint16_t seqCounter_ = 0;

    struct PendingRequest {
        bool active = false;
        bool fulfilled = false;
        uint8_t cmdId = 0;
        uint16_t seq = 0;
        void *out = nullptr;
        size_t outLen = 0;
    } pending_;

    FunctionFeedbackCallback feedbackCb_ = nullptr;

    bool debugEnabled_ = false;
    Print *debugOut_ = nullptr;

    uint32_t heartbeatIntervalMs_ = 0;
    uint32_t lastHeartbeatMs_ = 0;
};

/// Convenience alias matching the "SIYI camera(CameraSerial);" usage in
/// the brief -- default 64-byte payload capacity is enough for every
/// implemented command's largest ACK/request.
using SIYI = SIYIDriver<64>;

} // namespace siyi
