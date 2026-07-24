/**
 * @file Protocol.h
 * @brief SIYI External SDK protocol constants, command IDs, and payload
 *        layouts, transcribed from:
 *        "SIYI Gimbal Camera External SDK Protocol Document" (V0.1.1).
 *
 * Frame layout (Chapter 1):
 *
 *   Field    | Index | Bytes | Notes
 *   ---------|-------|-------|----------------------------------------
 *   STX      | 0     | 2     | 0x6655, low byte first  -> bytes {0x55,0x66}
 *   CTRL     | 2     | 1     | see note below
 *   Data_len | 3     | 2     | length of DATA field, low byte first
 *   SEQ      | 5     | 2     | frame sequence 0..65535, low byte first
 *   CMD_ID   | 7     | 1     | command ID
 *   DATA     | 8     | N     | command payload (N = Data_len)
 *   CRC16    | 8+N   | 2     | CRC16 over bytes [0 .. 8+N), low byte first
 *
 * CTRL byte ambiguity (documented, not guessed):
 *   The spec text defines CTRL as "0 = need_ack, 1 = ack_pack" (i.e. a
 *   packet marked 1 is itself an ACK reply). However the ONLY concrete
 *   worked example in the document -- the heartbeat frame
 *   "55 66 01 01 00 00 00 00 00 59 8B" -- is a host-to-device command
 *   frame and uses CTRL = 0x01, which contradicts that stated meaning.
 *   With no second worked example to resolve this, this driver follows
 *   the one byte-accurate example we do have and always transmits
 *   CTRL = kCtrlByte on outgoing frames. Request/response correlation is
 *   done via CMD_ID + SEQ, not via CTRL bit interpretation, so this
 *   ambiguity does not affect correctness of the driver.
 */
#pragma once

#include <stdint.h>

namespace siyi {

// ---------------------------------------------------------------------
// Frame-level constants
// ---------------------------------------------------------------------
constexpr uint8_t kStx0 = 0x55; ///< STX byte 0 (low byte of 0x6655)
constexpr uint8_t kStx1 = 0x66; ///< STX byte 1 (high byte of 0x6655)
constexpr uint8_t kCtrlByte = 0x01; ///< See CTRL ambiguity note above.

constexpr size_t kHeaderLen = 8;  ///< STX(2)+CTRL(1)+Data_len(2)+SEQ(2)+CMD_ID(1)
constexpr size_t kCrcLen = 2;
constexpr size_t kMinFrameLen = kHeaderLen + kCrcLen; ///< frame with 0-byte payload

// ---------------------------------------------------------------------
// Command IDs (Chapter 2) -- only commands relevant to gimbal/camera
// control on the A8 Mini are implemented. Thermal-imaging, laser
// rangefinder, GPS/RC telemetry and video-stitching commands (present in
// the document for the ZR10/ZR30/ZT30/quad-spectrum cameras) are outside
// the requested API surface and are intentionally not implemented here.
// ---------------------------------------------------------------------
enum class CommandId : uint8_t {
    Heartbeat         = 0x00, ///< No ACK.
    FirmwareVersion   = 0x01,
    HardwareId        = 0x02,
    AutoFocus         = 0x04,
    ManualZoom        = 0x05, ///< Manual zoom with autofocus.
    ManualFocus       = 0x06,
    GimbalRotation    = 0x07,
    CenterGimbal      = 0x08, ///< One-key centering.
    CameraSystemInfo  = 0x0A, ///< "Gimbal status" (record/HDR/motion mode/etc).
    FunctionFeedback  = 0x0B, ///< Unsolicited push from camera. No send.
    PhotoVideoControl = 0x0C, ///< Photo/record/motion-mode/HDMI. No ACK.
    GimbalAttitude    = 0x0D, ///< Request attitude (yaw/pitch/roll + rates).
    SetGimbalAttitude = 0x0E, ///< Absolute angle control.
    AbsoluteZoom      = 0x0F,
};

#pragma pack(push, 1)

// ---- 0x01 Firmware Version --------------------------------------------
struct FirmwareVersionAck {
    uint32_t camera_firmware_ver;
    uint32_t gimbal_firmware_ver;
    uint32_t zoom_firmware_ver;
    // Note (per spec): the 4th (high) byte of each field must be ignored.
    // Before ~30s post-boot the camera returns all-zero versions.
};

// ---- 0x02 Hardware ID ---------------------------------------------------
struct HardwareIdAck {
    uint8_t hardware_id[12]; ///< 10-digit ID string; first 2 bytes = product ID.
};

enum class ProductId : uint8_t {
    ZR10 = 0x6B,
    A8Mini = 0x73,
    A2Mini = 0x75,
    ZR30 = 0x78,
    QuadSpectrum = 0x7A,
};

// ---- 0x04 Auto Focus -----------------------------------------------------
struct AutoFocusSend {
    uint8_t auto_focus; ///< 1 = trigger single autofocus.
    uint16_t touch_x;
    uint16_t touch_y;
};
struct AutoFocusAck {
    uint8_t sta; ///< 1 = success, 0 = error
};

// ---- 0x05 Manual Zoom with Autofocus -------------------------------------
struct ManualZoomSend {
    int8_t zoom; ///< 1 = zoom in, 0 = stop, -1 = zoom out
};
struct ManualZoomAck {
    uint16_t zoom_multiple; ///< divide by 10 for actual zoom level
};

// ---- 0x06 Manual Focus ----------------------------------------------------
struct ManualFocusSend {
    int8_t focus; ///< 1 = far, 0 = stop, -1 = near
};
struct ManualFocusAck {
    uint8_t sta;
};

// ---- 0x07 Gimbal Rotation Control -----------------------------------------
struct GimbalRotationSend {
    int8_t turn_yaw;   ///< -100..100
    int8_t turn_pitch; ///< -100..100
};
struct GimbalRotationAck {
    uint8_t sta;
};

// ---- 0x08 One-Key Centering ------------------------------------------------
enum class CenterPos : uint8_t {
    OneKeyCenter  = 1,
    CenterDownward = 2,
    Center        = 3,
    Downward      = 4,
};
struct CenterGimbalSend {
    uint8_t center_pos;
};
struct CenterGimbalAck {
    uint8_t sta;
};

// ---- 0x0A Request Camera System Information ("Gimbal Status") -------------
enum class GimbalMotionMode : uint8_t {
    Lock = 0,
    Follow = 1,
    FPV = 2,
};
struct CameraSystemInfoAck {
    uint8_t reserved1;
    uint8_t hdr_sta;             ///< 0 = off, 1 = on
    uint8_t reserved2;
    uint8_t record_sta;          ///< 0 not recording,1 recording,2 no TF,3 data loss
    uint8_t gimbal_motion_mode;  ///< see GimbalMotionMode
    uint8_t gimbal_mounting_dir; ///< 0 reserved,1 normal,2 inverted
    uint8_t video_hdmi_or_cvbs;  ///< 0 = HDMI on/CVBS off, 1 = HDMI off/CVBS on
    uint8_t zoom_linkage;        ///< 0 = off, 1 = on
};

// ---- 0x0C Capture Photo / Record Video / Mode / HDMI ----------------------
enum class FuncType : uint8_t {
    CapturePhoto     = 0,
    HdrToggle        = 1, ///< not supported (per spec)
    StartRecording   = 2,
    LockMode         = 3,
    FollowMode       = 4,
    FpvMode          = 5,
    EnableHdmi       = 6,  ///< reboot required
    EnableCvbs       = 7,  ///< reboot required
    DisableHdmiCvbs  = 8,  ///< reboot required
    TiltDownward     = 9,
    ZoomLinkage      = 10,
};
struct PhotoVideoControlSend {
    uint8_t func_type;
    // No ACK for this command (per spec). Async status arrives via
    // CMD_ID 0x0B (FunctionFeedback), which this driver exposes through
    // an optional user callback rather than a request/response call.
};

// ---- 0x0B Function Feedback (unsolicited, no send) -------------------------
enum class FunctionFeedbackInfo : uint8_t {
    PhotoSuccess       = 0,
    PhotoFailed        = 1,
    HdrOn              = 2,
    HdrOff             = 3,
    RecordingFailed    = 4,
    RecordingStarted   = 5,
    RecordingStopped   = 6,
};
struct FunctionFeedbackAck {
    uint8_t info_type;
};

// ---- 0x0D Request Gimbal Attitude Data --------------------------------------
struct GimbalAttitudeAck {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t yaw_velocity;
    int16_t pitch_velocity;
    int16_t roll_velocity;
    // Note (per spec): divide all fields by 10 for real values (1 decimal).
};

// ---- 0x0E Set Gimbal Attitude Angles (absolute angle control) --------------
struct SetGimbalAttitudeSend {
    int16_t yaw;   ///< target yaw *10 (e.g. 60.5 deg -> 605)
    int16_t pitch; ///< target pitch *10
};
struct SetGimbalAttitudeAck {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
};

// A8 Mini angle range limits (per spec Appendix note under 0x0E):
constexpr float kA8MiniYawMinDeg = -135.0f;
constexpr float kA8MiniYawMaxDeg = 135.0f;
constexpr float kA8MiniPitchMinDeg = -90.0f;
constexpr float kA8MiniPitchMaxDeg = 25.0f;

// ---- 0x0F Absolute Zoom Auto Focus ------------------------------------------
struct AbsoluteZoomSend {
    uint8_t absolute_movement_int;   ///< integer part of zoom, 0x1..0x1E
    uint8_t absolute_movement_float; ///< decimal part of zoom, 0x0..0x9
};
struct AbsoluteZoomAck {
    uint8_t absolute_movement_ask; ///< 1 = success
};

#pragma pack(pop)

} // namespace siyi
