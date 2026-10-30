# ESP32-CAM Face Tracking Car

A small robot car that finds a human face and drives toward it, with **all face detection running on the ESP32-CAM itself**. No cloud, no PC, no external AI service. The browser is only used to watch the video and to drive manually.

## Features

- Live MJPEG video stream with a green box drawn around the detected face
- On-device two-stage face detection (Espressif esp-dl)
- Automatic tracking: turns toward the face and drives forward until it is close
- Manual driving from the browser (WASD keys or touch buttons)
- Detection runs on its own CPU core, so the video stays smooth

## Hardware

- AI Thinker ESP32-CAM (with PSRAM)
- H-bridge motor driver and two DC motors
- 5 V supply that can handle motor current (the board browns out easily on USB power alone)

| Function | GPIO |
|---|---|
| Left motor IN1 / IN2 | 12 / 13 |
| Right motor IN1 / IN2 | 14 / 15 |

Note: GPIO 12 is a boot strapping pin on the ESP32. If the board fails to boot with the driver connected, disconnect that wire during power-up or move it to another pin.

## Software setup

1. Arduino IDE with **ESP32 by Espressif, version 2.0.17** (the face detection headers used here are part of the 2.0.x core; they are not available in 3.x).
2. Board: **AI Thinker ESP32-CAM**
3. Partition scheme: **Huge APP (3MB No OTA / 1MB SPIFFS)**
4. PSRAM: enabled
5. Edit the WiFi credentials at the top of the sketch:

```cpp
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
```

6. Upload, open the Serial Monitor at 115200 baud, and note the IP address it prints.
7. Open `http://<that-ip>/` in a browser on the same network.

## Usage

- The page opens in **Auto-Tracking** mode. The car looks for a face and follows it.
- Press W / A / S / D (or the on-screen buttons) to drive manually. This switches to manual mode automatically.
- If no manual command arrives for 1 second, the car returns to auto-tracking and stops the motors.

HTTP endpoints:

| Endpoint | Port | Purpose |
|---|---|---|
| `/` | 80 | Control page |
| `/control?var=...&val=...` | 80 | detect on/off, mode, car command, face count |
| `/status` | 80 | JSON: detect state, manual mode, face count |
| `/capture` | 80 | Single JPEG snapshot |
| `/stream` | 81 | MJPEG video stream |

## How it works

### Two cores, two jobs

- **Core 1** runs the stream server. For each frame it draws the latest face box, compresses to JPEG and sends it to the browser.
- **Core 0** runs `detectTask`. When it wants to detect, it asks the stream for a frame. The stream copies one into a separate buffer, and detection runs on that copy.

Because detection is slow compared to streaming, keeping it on its own core means the video never waits for it.

### Tracking logic

1. Detection returns zero or more face boxes. The largest one is used.
2. If the face centre is left of the image centre by more than 35 px, turn right. If right by more than 35 px, turn left.
3. If the face is centred but its height is under 95 px (so it is far away), drive forward.
4. Each correction is a short 40 ms motor pulse, followed by a 350 ms pause so the image settles before the next detection.

## How face detection runs locally on the ESP32

It sounds impossible for a $10 board, but a few things make it work:

- **Capable hardware.** The ESP32 has two cores at up to 240 MHz, and the AI Thinker board adds external PSRAM, which holds the camera frames and the neural network's working memory.
- **Small models.** The detector is a pair of compact neural networks from Espressif's esp-dl library, built for microcontrollers (compressed, low-precision weights and optimized math).
- **Two stages.** Stage 1 (`HumanFaceDetectMSR01`) scans the whole image quickly and proposes candidate face regions. Stage 2 (`HumanFaceDetectMNP01`) re-checks only those small regions and refines the boxes. This is cheaper than running one big network over the full frame.
- **Small input.** Frames are only 320×240 pixels, so there is little data to process.
- **Detection, not recognition.** The model only answers "is there a face and where". It does not identify who it is, which keeps it small.

The result: the camera frame goes in, face boxes come out, and nothing ever leaves the chip. Detection is limited to a few updates per second, which is plenty for steering a slow robot.

## Tuning

| Setting | Where | Effect |
|---|---|---|
| `JPEG_QUALITY` | top of file | Lower = faster stream, worse picture |
| `corridor_margin` (35) | `detectTask` | Width of the "centred" dead zone |
| `face_h < 95` | `detectTask` | How close the car gets before stopping |
| `MOVE_DURATION` (40 ms) | top of file | Length of each motor pulse |
| `COOLDOWN_DELAY` (350 ms) | top of file | Pause between detections |
| Detector thresholds | `initDetector()` | Higher = fewer false positives, fewer detections |

## Troubleshooting

- **Brownout / random resets:** power the board from a stable 5 V supply, not just USB.
- **Camera init failed:** check the ribbon cable and the pin map.
- **Header not found (`human_face_detect_msr01.hpp`):** you are not on ESP32 core 2.0.x.
- **No video in the browser:** make sure port 81 is reachable on your network.

## Credits

- Face detection models and library: [Espressif esp-dl / esp-who](https://github.com/espressif/esp-who)
- Camera driver, HTTP streaming pattern and the AI Thinker pin map: [Espressif arduino-esp32](https://github.com/espressif/arduino-esp32) (CameraWebServer example)
- Tracking and car control logic: this project

## License

Add your preferred license here. Check the licenses of the Espressif components you build on.
