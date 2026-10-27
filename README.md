# BlueberryVR

BlueberryVR is a SteamVR driver for building your own VR headset. It tracks the headset's position with a single camera mounted on the headset, so you don't need base stations or external cameras. It also supports DIY wireless controllers.

It is a fork of [HadesVR](https://github.com/HadesVR/HadesVR). The original PSMoveService tracking has been replaced with inside-out camera tracking. Some internals, such as the `driver_HadesVR` project name, still carry the HadesVR name.

![Headset](docs/img/Headset.png)

## How it works

| Part | Source |
| --- | --- |
| Headset rotation | IMU in the headset, filtered with Madgwick |
| Headset position | Monocular SLAM on the headset camera: ORB features, PnP against a persistent map, local bundle adjustment |
| Controller rotation, buttons, fingers | Controller IMUs and inputs, sent over RF to a receiver built into the headset |
| Controller position | The headset camera finds the coloured LED balls on the controllers; the ball's size and image position give its 3D position |

The SLAM runs inside the driver, so there is nothing else to install or start before SteamVR. The camera's lens parameters and mounting rotation are learned automatically while you use the headset, and the room map is saved between sessions.

Positions are filtered with Kalman filters, optionally fused with the IMUs' accelerometers. See the [tracking docs](docs/Tracking.md) for setup and limitations.

## Modes

- **Headset and controllers:** full 6DoF tracking for the headset and both controllers.
- **Headset only:** 6DoF from the camera, or 3DoF if you set `EnableSlam` to `false`.
- **Controllers only:** use the controllers with another headset. You'll need an [RF receiver](docs/RFReceiver.md).

## Features and limits

- Emulates HTC Vive wands, and Valve Index controllers including finger tracking.
- Controllers are only tracked while the headset camera can see them. A wide-angle lens (100° or more) helps a lot.
- A single camera can't know the room's size. The scale is measured from the controller LEDs, so it's a guess until a controller is held still in view while you move your head.
- The SLAM is small and has no loop closure, so drift is only corrected by re-seeing mapped places. It needs a lit room with some visual texture.
- Full body tracking is not implemented. The trackers section of the driver does nothing yet.
- The driver is Windows only. It needs SteamVR, uses hidapi or a serial port to talk to the headset, and OpenCV (via vcpkg) for the SLAM.

## Getting started

1. Build or assemble the hardware: [headset](docs/Headset.md), [controllers](docs/Controllers.md) and, if needed, an [RF receiver](docs/RFReceiver.md).
2. Flash the firmware from `Software/Firmware/`.
3. Install the driver into `Steam\steamapps\common\SteamVR\drivers` and configure it. See the [driver docs](docs/Driver.md).
4. Mount a webcam on the headset and set up tracking. See the [tracking docs](docs/Tracking.md).
5. Start SteamVR. Move the headset sideways 10-30 cm while looking at a textured part of the room so the SLAM can start mapping. Look around for the first half minute so the camera calibration can settle.
6. Press F8 to calibrate the headset and controller yaw.

All documentation is indexed in [docs/DocsIndex.md](docs/DocsIndex.md).

## Repository layout

| Path | Contents |
| --- | --- |
| `Software/Driver/` | The SteamVR driver (C++), its resources and settings, and a SLAM simulator test in `tests/` |
| `Software/Firmware/` | Arduino firmware for the headset, receiver and controllers (HTC wand and Index styles) |
| `Hardware/Controllers/` | Controller PCBs and 3D printable shells |
| `Basic-HMD-PCB-main/` | PCB for the basic HMD tracking electronics |
| `docs/` | Build, configuration and tracking guides |

## Testing the SLAM without hardware

`Software/Driver/tests/slam_sim.cpp` flies a virtual camera through a synthetic textured room with a virtual LED. It feeds the frames to the real `SlamTracker` and prints the error against ground truth. Run `Software/Driver/tests/run_slam_sim.bat` from a normal command prompt. See [Testing without hardware](docs/Tracking.md#testing-without-hardware) for the arguments.

## Hardware status

- The Basic HMD and the Wand controllers don't have their 3D printable parts yet.
- The custom Index (Knuckles) controllers are not recommended. They have known board and shell problems, the capacitive finger sensors are unreliable, and the boards make flashing the bootloader difficult. Index-style controllers built from regular Arduino boards are a better option; see the [controller docs](docs/Controllers.md).

## Changelog

Major features and changes, newest first.

- **SLAM speed and accuracy pass** (not yet verified on hardware or in the simulator):
  - The camera pose is predicted with a constant-velocity guess, so the map search lands closer to where the features are during fast head motion.
  - After the RANSAC pose estimate, only matches that agree within 2 px are kept and the pose is refined again, so a few wrong matches no longer pull it off.
  - Relocalization after tracking loss matches against the map on all CPU cores.
  - LED detection skips contour processing on frames where no pixel matches the LED colour.
- Automatic camera calibration: the lens and camera mounting rotation are learned while running and saved.
- Local bundle adjustment and LED-based scale estimation, plus the synthetic SLAM simulator.
- Replaced PSMoveService with headset camera SLAM and LED tracking.

## Credits and license

BlueberryVR is based on [HadesVR](https://github.com/HadesVR/HadesVR), which was in turn derived from [TrueOpenVR](https://github.com/TrueOpenVR). See [LICENSE](LICENSE).
