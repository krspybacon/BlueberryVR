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
3. If the camera is not mounted perfectly level, set `CameraPitchAngle` (positive = tilted up), `CameraYawAngle`, `CameraRollAngle` in degrees.
4. Set `HMDXOffset/HMDYOffset/HMDZOffset` (in the `"HMD"` section) to the offset from the **camera lens to the center of your head**. These work exactly like the old tracking ball offset did.
5. Set `DebugImage` to something like `C:/Users/you/Desktop/slam.jpg`. The driver writes an annotated picture twice per second which is the easiest way to see what the camera sees: green dots are features, orange rings are features that are part of the map, circles around the LEDs show what was recognised as a controller.

### Camera calibration (important)
Without calibration the driver assumes an ideal pinhole camera with `HorizontalFOV` degrees of view. It works, but lens distortion makes the tracking drift and swim. For good results calibrate your camera once with OpenCV's chessboard calibration (`calibrateCamera`, the OpenCV docs have a tutorial) at the resolution you use, and copy the numbers into `Fx`, `Fy`, `Cx`, `Cy` and `DistortionK1/K2/P1/P2/K3`.

### Controller LEDs
- `LEDRightHue` and `LEDLeftHue` are OpenCV hues, 0-179 (red 0, yellow 30, green 60, cyan 90, blue 120, magenta 150). Pick the ones that match your LEDs, they must be at least ~30 apart.
- `LEDHueTolerance`, `LEDMinSaturation`, `LEDMinValue` control how strict the colour match is.
- Webcams love to blow out a bright LED into a white blob, which hides its colour. Turn `AutoExposure` off and lower `Exposure` (try -6, -7, -8) until the LED is a small, solid, saturated colour on a darker image.
- `LEDDiameter` is the diameter of your ball in meters (a ping pong ball is 0.04). If the controllers appear too close or too far, tune `LEDDepthScale` (the glow of the LED makes the ball look bigger than it is, so you will usually need a value slightly above 1).

# How the room gets mapped
- When SteamVR starts, the driver is in **INIT** mode. Look at a textured part of the room and **move the headset sideways by 10-30cm** (don't just rotate it). Once there is enough parallax the first map is created and tracking starts.
- **Scale**: a single camera cannot know how big the room is. The driver gets real-world scale from the controller LEDs: if a controller is held still in view during initialization, its known ball size sets the scale of the map. If no LED is visible it falls back to assuming the scene is `FallbackSceneDepth` meters away on average, which may be off, and then your world will feel too big or too small. **Hold a controller in view, fairly still, while you do the initial sideways move.**
- While you play, the map grows as you look around. If tracking is lost (covering the camera, going into a blank area) the driver searches for the known map and picks the tracking back up as soon as it recognises where it is.
- The map is saved to `MapFile` when SteamVR closes and loaded next time, so your play space, its scale and its origin stay the same between sessions. Delete the file to start over. Changing the camera mounting, intrinsics or resolution requires deleting the old map.
- The headset's starting position is placed at `OriginX/Y/Z` in the SteamVR world (default 1.6m above the floor). Tweak `OriginY` to set your height, or use SteamVR's room setup.

# Limitations
This is a deliberately small SLAM, not ORB-SLAM3 or Quest-grade tracking:
- It is monocular, scale comes from the LEDs and can drift over long sessions.
- No bundle adjustment and no loop closure, so drift is only corrected by re-seeing mapped places.
- Controllers are only tracked while visible to the headset camera; outside of it the IMU (if `UseAccelerometers` is on) takes over, which drifts quickly.
- Motion blur and rolling shutter will hurt it. Good light and a short exposure help a lot.

# Controller calibration
Controller and headset *rotation* calibration (F8, F7 to reset) works the same as before, see the [Driver docs](Driver.md#usage-and-controller-bindings).
