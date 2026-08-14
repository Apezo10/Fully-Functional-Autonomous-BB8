BB-8 Bluetooth Connection

This project documents the design and integration of a BB-8 inspired robotic ball that combines embedded motion control, Bluetooth communication, and computer vision based tracking. The central goal is to move from a static spherical shell to a responsive robotic system that can be controlled, observed, and evaluated through both hardware behavior and visual feedback.

The build is organized around an ESP32 development board using the Arduino framework in PlatformIO. The repository also includes a Bluepad32 template, which supports Bluetooth controller input and provides a practical foundation for pairing gamepads or other HID devices with the robot. On the perception side, the project uses a MobileNet object detector with a KCF tracker to identify the ball and follow its position through video frames.

## Demonstration

### MobileNet + KCF Detection and Tracking

The first test validates the perception pipeline. MobileNet performs object detection, while KCF maintains frame-to-frame continuity after the target is located. This pairing is useful because detection alone can be computationally expensive, whereas tracking alone can drift without a reliable initialization step. Together, they produce a more stable and efficient vision loop.

<video src="media/mobilenet-kcf-demo.mp4" controls muted playsinline preload="metadata" width="720"></video>

[Watch the MobileNet + KCF detection and tracking video](media/mobilenet-kcf-demo.mp4)

### Integrated Ball Motion

The second test shows the assembled ball moving through a half-hemisphere range of motion. This demonstration is important because it moves the project beyond isolated electronics and confirms that the internal actuation system can translate control input into visible mechanical behavior.

<video src="media/integrated-ball-motion.mp4" controls muted playsinline preload="metadata" width="720"></video>

[Watch the integrated ball motion video](media/integrated-ball-motion.mp4)

## Project Objectives

The project was developed to satisfy four technical objectives:

1. Build a spherical robotic platform capable of controlled motion.
2. Establish Bluetooth communication between a controller and an ESP32.
3. Integrate embedded firmware with the mechanical movement system.
4. Evaluate the robot through video-based detection, tracking, and motion analysis.

These objectives make the project a useful study in mechatronics because it requires coordination between software, electronics, mechanical design, and computer vision rather than treating each subsystem as an isolated exercise.

## System Overview

The robot can be understood as a set of interacting subsystems:

| Subsystem | Purpose |
| --- | --- |
| ESP32 firmware | Reads control input and coordinates the robot's embedded behavior. |
| Bluetooth communication | Enables wireless control through the Bluepad32 framework. |
| Mechanical assembly | Converts internal actuation into rolling motion of the spherical body. |
| Vision pipeline | Uses MobileNet and KCF to detect and track the ball during testing. |
| Evaluation workflow | Compares expected movement against observed range of motion. |

The overall architecture follows a feedback-oriented design. User input drives the embedded controller, the controller actuates the ball, and the external camera pipeline verifies whether the physical system behaves as intended.

## Technical Approach

### 1. Embedded Platform

The firmware is configured for an ESP32 development board through PlatformIO:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
```

This configuration was selected because the ESP32 provides Bluetooth capability, adequate embedded performance, and broad support through the Arduino and PlatformIO ecosystems. PlatformIO also makes the project easier to reproduce because board configuration, build settings, and upload behavior are stored in a single project file.

### 2. Bluetooth Control

The `bluepad32-template` directory provides the Bluetooth controller foundation. Bluepad32 is designed for gamepad and HID connectivity, which makes it appropriate for robotic control where low-latency directional input is required. In this project, Bluetooth communication functions as the bridge between human command and embedded response.

### 3. Motion Integration

The mechanical system is evaluated through its ability to move the ball through a half-hemisphere range. That range of motion is a meaningful milestone because spherical robots must manage a difficult relationship between internal actuation, center of mass, surface friction, and shell geometry. A successful integrated motion test indicates that the mechanical and electrical subsystems are working together rather than merely functioning independently.

### 4. CAD and Mechanical Design

The mechanical model for the BB-8 inspired body was developed in Onshape. The CAD file documents the physical geometry of the spherical structure and provides a reproducible reference for the mechanical layout used during integration.

[View the Onshape CAD model](https://cad.onshape.com/documents/41da30e1bcacc968cfe0cdbb/w/b265e77dee8ec4512725b7e4/e/810610f244652ecffd3517d6?renderMode=0&uiState=6a7e6e03cec9f2cae196ff60)

### 5. Computer Vision Evaluation

The perception workflow combines MobileNet and KCF:

| Method | Role in the pipeline |
| --- | --- |
| MobileNet | Performs object detection and provides an initial target location. |
| KCF | Tracks the detected target across subsequent frames. |

MobileNet contributes semantic recognition, while KCF contributes temporal continuity. This division of labor is computationally sensible because the detector can identify the object, and the tracker can follow it efficiently without requiring a full detection pass on every frame.

## Repository Structure

```text
BB8 Bluetooth Connection/
  include/                 Header files for the PlatformIO project
  lib/                     Project-specific libraries
  src/                     Main firmware source directory
  test/                    PlatformIO test directory
  media/                   Project demonstration videos
  bluepad32-template/      Bluetooth controller template and dependencies
  platformio.ini           ESP32 board and framework configuration
```

The mechanical CAD model is hosted externally in Onshape and linked above so the design can be inspected alongside the embedded project files.

## Getting Started

### Prerequisites

Install the following before building or flashing the firmware:

- Visual Studio Code
- PlatformIO extension
- ESP32 development board
- USB data cable
- Compatible Bluetooth controller, if using controller input

### Build the Firmware

Open the project folder in VS Code, then build through the PlatformIO toolbar. The same operation can be performed from a terminal:

```bash
pio run
```

### Upload to the ESP32

Connect the ESP32 by USB and run:

```bash
pio run --target upload
```

If the upload port is not detected automatically, specify it in `platformio.ini` or select it from the PlatformIO device menu.

### Monitor Serial Output

Serial output is useful for verifying startup behavior, controller pairing, and runtime diagnostics:

```bash
pio device monitor
```

## Testing and Validation

The project was evaluated in two major stages.

First, the vision system was tested with the MobileNet + KCF pipeline. This confirmed that the robot could be detected and tracked visually, which provides an empirical basis for later motion analysis.

Second, the assembled ball was tested for physical movement. The half-hemisphere demonstration verifies that the internal mechanism can generate meaningful range of motion inside the spherical shell.

Together, these tests show both observational reliability and mechanical functionality. The robot is not only capable of movement; its movement can also be examined through a repeatable visual method.

## Design Rationale

The project uses a modular design because robotic systems become difficult to debug when every part is tested only after final assembly. By separating Bluetooth input, embedded firmware, mechanical motion, and visual tracking, each component can be developed and evaluated with clearer expectations.

MobileNet and KCF were paired for similar reasons. MobileNet supplies robust detection, while KCF supplies efficient tracking. This balances accuracy with computational practicality, which is especially important when robotics projects must operate under hardware, timing, and power constraints.

## Current Status

The repository currently contains the PlatformIO project structure, ESP32 board configuration, and Bluepad32 template resources. The demonstration videos document progress in perception testing and integrated mechanical motion. Future commits can expand the firmware in `src/main.cpp`, add wiring diagrams, document controller mappings, and include calibration data from motion trials.

## Future Work

Potential improvements include:

- Implementing finalized ESP32 control logic in `src/main.cpp`
- Documenting the motor driver and wiring layout
- Adding controller input mappings
- Recording quantitative tracking results from the MobileNet + KCF pipeline
- Measuring response time, drift, turning radius, and repeatability
- Exporting the Onshape CAD model as `.step`, `.stl`, or assembly drawings for direct archival in the repository

## Conclusion

This BB-8 robotic ball project demonstrates the early integration of embedded control, wireless communication, mechanical actuation, and computer vision. Its strength lies in the fact that the system is evaluated both internally, through controller and firmware behavior, and externally, through visual tracking. That combination creates a more disciplined engineering workflow and gives the project a clear path from prototype to more rigorous robotic experimentation.

