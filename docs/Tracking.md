# Tracking
Positional tracking is done with **one camera mounted on the headset** (inside-out), no base stations or external cameras.

The driver does two things with that camera:
- **SLAM**: it detects ORB features in the room, builds a sparse 3D map of them and works out where the camera is by matching the live image against that map. That gives the 3D position of the headset. The orientation of the headset still comes from its IMU, the SLAM uses it to align the map to gravity and to predict fast head turns.
- **LED detection**: the glowing balls (ping pong ball + coloured LED) on the controllers are found in the image by colour. Their size in the image gives their distance, their position in the image gives their direction. The controllers' orientation still comes from their IMUs.

Everything runs inside the driver, there is nothing else to install or start before SteamVR.

# What you'll need
- A USB webcam on the headset, facing forward. Wide field of view (100°+), global shutter or at least a fast rolling shutter, and 60fps at 720p or 640x480 are what you want. Controllers are only tracked while the camera can see them, so a wide lens matters a lot.
- Controllers with a ping pong ball + LED each. Use two **clearly different, saturated colours**, for example magenta and cyan.
- A room that is lit and has texture (posters, furniture, clutter). Plain white walls and dark rooms give the SLAM nothing to latch onto.

# Camera setup
All settings are in the `"Slam"` section of `default.vrsettings`, see the [Driver docs](Driver.md#slam-section).

1. Mount the camera rigidly to the headset. If it flexes, the tracking will wobble.
2. Set `CameraIndex` (0 is the first camera Windows lists), `CameraWidth`, `CameraHeight` and `CameraFps`.
3. Nothing else is needed for the camera itself: the lens and the way the camera is mounted are learned automatically, see [Automatic calibration](#automatic-calibration).
4. Set `HMDXOffset/HMDYOffset/HMDZOffset` (in the `"HMD"` section) to the offset from the **camera lens to the center of your head**. These work exactly like the old tracking ball offset did.
5. Set `DebugImage` to something like `C:/Users/you/Desktop/slam.jpg`. The driver writes an annotated picture twice per second which is the easiest way to see what the camera sees: green dots are features, orange rings are features that are part of the map, circles around the LEDs show what was recognised as a controller.

### Automatic calibration
You do **not** need to calibrate the camera or measure how it is mounted. While you use the headset the driver learns:
- **The lens**: focal length, centre of the image and barrel distortion, by treating them as unknowns in the SLAM's bundle adjustment. It starts from `HorizontalFOV` as a rough guess (anything within ~30% is fine).
- **How the camera is mounted** (its rotation relative to the headset's IMU), by comparing how the camera and the IMU saw the head turn. For this you need to **look around, turning your head left/right and up/down**; after a few turns (typically 20-30 seconds) the mounting is found and the log says `Camera mounting rotation updated`.

Until both are learned, positions can be a bit off or swim slightly, so just look around for the first half minute. The result is saved to `CalibrationFile` when SteamVR closes and loaded on the next start, so after the first session you start calibrated. **Delete that file if you change the camera, its resolution or how it is mounted.**

If you do have a proper calibration (OpenCV chessboard), you can still enter it in `Fx`, `Fy`, `Cx`, `Cy`, `DistortionK1/K2/P1/P2/K3` and the driver will trust it more and only polish it. Set `AutoCalibrate` to `false` to turn all learning off. The camera mounting angles `CameraPitchAngle/YawAngle/RollAngle` then become your fixed values; with it on they are just the starting guess.

What is **not** learned automatically: the LED glow bias (see `LEDDepthScale` below), the LED hues and the exposure.

### Controller LEDs
- `LEDRightHue` and `LEDLeftHue` are OpenCV hues, 0-179 (red 0, yellow 30, green 60, cyan 90, blue 120, magenta 150). Pick the ones that match your LEDs, they must be at least ~30 apart.
- `LEDHueTolerance`, `LEDMinSaturation`, `LEDMinValue` control how strict the colour match is.
- Webcams love to blow out a bright LED into a white blob, which hides its colour. Turn `AutoExposure` off and lower `Exposure` (try -6, -7, -8) until the LED is a small, solid, saturated colour on a darker image.
- `LEDDiameter` is the diameter of your ball in meters (a ping pong ball is 0.04). The LED's distance (and so the map scale) comes from its size in the image, and LED glow makes the ball look bigger than it is. Calibrate `LEDDepthScale`: hold a controller exactly 0.5m from the camera lens, read the distance written next to it in the `DebugImage`, and set `LEDDepthScale` = 0.5 / what it shows.
- A real LED glows, so the driver only accepts blobs that have a near-saturated bright core. Colourful posters/clothes of the same hue are usually ignored, but pick hues that are rare in your room.

# How the room gets mapped
- When SteamVR starts, the driver is in **INIT** mode. Look at a textured part of the room and **move the headset sideways by 10-30cm** (don't just rotate it). Once there is enough parallax the first map is created and tracking starts.
- **Scale**: a single camera cannot know how big the room is. At startup the driver assumes the scene is `FallbackSceneDepth` meters away on average, which is only a rough guess. The real scale is then measured continuously from the controller LEDs: whenever a controller is held **still** in view while you move your head around (a second or two of walking or leaning is enough), the known ball size together with your head motion gives the real scale, and the whole map is corrected. Measurements where the controller was moving are rejected automatically. The console log shows `Map scale measured/corrected` when this happens. Until the first measurement the world can feel too big or too small.
- While you play, the map grows as you look around. If tracking is lost (covering the camera, going into a blank area) the driver searches for the known map and picks the tracking back up as soon as it recognises where it is.
- The map is saved to `MapFile` when SteamVR closes and loaded next time, so your play space, its scale and its origin stay the same between sessions. Delete the file to start over. Changing the camera mounting, intrinsics or resolution requires deleting the old map.
- The headset's starting position is placed at `OriginX/Y/Z` in the SteamVR world (default 1.6m above the floor). Tweak `OriginY` to set your height, or use SteamVR's room setup.

# Limitations
This is a deliberately small SLAM, not ORB-SLAM3 or Quest-grade tracking:
- It is monocular: scale comes from the LEDs, and without a still controller in view it is just a guess.
- Local bundle adjustment keeps the drift low, but there is no loop closure, so drift is only corrected by re-seeing mapped places.
- The IMU is only used for orientation. Fusing its accelerometer would help in fast motion and with scale, this is not done yet.
- Controllers are only tracked while visible to the headset camera; outside of it the IMU (if `UseAccelerometers` is on) takes over, which drifts quickly.
- Motion blur and rolling shutter will hurt it. Good light and a short exposure help a lot.

# Testing without hardware
`Software/Driver/tests/slam_sim.cpp` renders a synthetic textured room, flies a virtual camera through it (with a virtual LED), feeds the frames to the real `SlamTracker` and prints the error against ground truth. Run `Software/Driver/tests/run_slam_sim.bat [speed] [frames] [led 0/1] [imuNoiseDeg] [pixelNoise] [blackoutStartFrame] [blackoutFrames] [calibMode] [calibrationFile]` (calibMode 1, the default, gives the tracker a wrong lens guess and an unknown camera mounting so it has to learn them; 0 gives it the right ones) from a normal command prompt (needs the vcpkg OpenCV, see the batch file). It is the quickest way to check that a change to the SLAM did not make it worse.

# Controller calibration
Controller and headset *rotation* calibration (F8, F7 to reset) works the same as before, see the [Driver docs](Driver.md#usage-and-controller-bindings).
