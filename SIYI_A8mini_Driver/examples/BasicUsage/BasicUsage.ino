/**
 * BasicUsage.ino
 *
 * Demonstrates the SIYI A8 Mini driver on an ESP32.
 *
 * Wiring: connect the gimbal's UART (Gimbal Control port, TX/RX/GND) to
 * an ESP32 HardwareSerial UART. Adjust RX_PIN / TX_PIN for your wiring.
 */
#include <SIYI.h>

using namespace siyi;

// Any free HardwareSerial port (UART2 on most ESP32 boards).
HardwareSerial CameraSerial(2);
SIYI camera(CameraSerial);

constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;

void onFeedback(FunctionFeedbackInfo info) {
    switch (info) {
        case FunctionFeedbackInfo::PhotoSuccess:     Serial.println("Photo captured"); break;
        case FunctionFeedbackInfo::PhotoFailed:      Serial.println("Photo failed (check TF card)"); break;
        case FunctionFeedbackInfo::RecordingStarted: Serial.println("Recording started"); break;
        case FunctionFeedbackInfo::RecordingStopped: Serial.println("Recording stopped"); break;
        case FunctionFeedbackInfo::RecordingFailed:  Serial.println("Recording failed (check TF card)"); break;
        case FunctionFeedbackInfo::HdrOn:            Serial.println("HDR on"); break;
        case FunctionFeedbackInfo::HdrOff:           Serial.println("HDR off"); break;
    }
}

void setup() {
    Serial.begin(115200);

    CameraSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    camera.begin();

    // Optional: verbose TX/RX/CRC/timeout logging to the USB serial port.
    camera.setDebug(true, Serial);

    // Get notified of async photo/record/HDR results (CMD 0x0B).
    camera.onFunctionFeedback(onFeedback);

    // --- Query commands (blocking, bounded timeout) ---
    FirmwareVersionAck fw{};
    if (camera.getFirmwareVersion(fw, 500)) {
        Serial.printf("Camera FW: 0x%06X  Gimbal FW: 0x%06X  Zoom FW: 0x%06X\n",
                       fw.camera_firmware_ver & 0xFFFFFF,
                       fw.gimbal_firmware_ver & 0xFFFFFF,
                       fw.zoom_firmware_ver & 0xFFFFFF);
    } else {
        Serial.println("Firmware version request timed out");
    }

    HardwareIdAck hwid{};
    if (camera.getHardwareId(hwid, 500)) {
        Serial.print("Hardware ID: ");
        for (int i = 0; i < 12; i++) Serial.printf("%02X", hwid.hardware_id[i]);
        Serial.println();
        if (hwid.hardware_id[0] == static_cast<uint8_t>(ProductId::A8Mini)) {
            Serial.println("Confirmed: A8 Mini");
        }
    }

    // One-key center on startup.
    camera.center();

    // Put the gimbal into Follow mode.
    camera.setMotionMode(GimbalMotionMode::Follow);
}

void loop() {
    // MUST be called frequently: pumps UART RX, parses frames, fulfills
    // pending blocking requests, dispatches async callbacks.
    camera.update();

    static uint32_t lastAction = 0;
    uint32_t now = millis();

    if (now - lastAction > 5000) {
        lastAction = now;

        // Move to an absolute angle within the A8 Mini's supported range.
        SetGimbalAttitudeAck att{};
        if (camera.setAngle(30.0f, -15.0f, att, 500)) {
            Serial.printf("Gimbal now at yaw=%.1f pitch=%.1f roll=%.1f\n",
                           att.yaw / 10.0f, att.pitch / 10.0f, att.roll / 10.0f);
        }

        // Poll gimbal status.
        CameraSystemInfoAck status{};
        if (camera.getGimbalStatus(status, 500)) {
            Serial.printf("record_sta=%u motion_mode=%u hdr=%u\n",
                           status.record_sta, status.gimbal_motion_mode, status.hdr_sta);
        }

        // Take a photo (no ACK per protocol; result comes via onFeedback()).
        camera.takePhoto();
    }
}
