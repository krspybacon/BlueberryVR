#pragma once

#ifndef _SlamTracker_
#define _SlamTracker_

#include <atomic>
#include <functional>
#include <string>
#include <thread>

/**
	Headset mounted camera tracking. A single (monocular) camera is used for two things:
	 - Sparse feature SLAM (ORB features, PnP tracking against a persistent map) which gives the position of the camera in the room.
	 - Detection of the glowing LED spheres on the controllers, which gives their position relative to the camera.

	Orientation is NOT estimated here, the IMU already provides it. The SLAM only needs an orientation prior to
	align its map with gravity / the headset's yaw and to help prediction during fast head turns.

	All output positions are in SteamVR world space: meters, +Y up, -Z forward, right handed.
*/

struct SlamConfig {
	// camera
	int		cameraIndex = 0;
	int		width = 1280;
	int		height = 720;
	int		fps = 60;
	bool	autoExposure = true;
	double	exposure = -6.0;				// only used if autoExposure is false, driver dependent (DirectShow uses log2 seconds)
	double	fx = 0, fy = 0, cx = 0, cy = 0;	// intrinsics in pixels at width x height. fx = 0 means "estimate from hfov"
	double	hfovDeg = 90.0;
	double	dist[5] = { 0, 0, 0, 0, 0 };	// k1 k2 p1 p2 k3

	// camera mounting relative to the headset IMU frame (degrees). pitch > 0 tilts the camera up.
	double	camPitchDeg = 0, camYawDeg = 0, camRollDeg = 0;

	// LED detection (OpenCV hue range is 0-179). Index 0 = right controller, 1 = left controller.
	double	ledDiameter = 0.04;				// meters
	double	ledDepthScale = 1.0;			// multiplier on the computed LED distance, to compensate for LED glow
	int		ledHue[2] = { 150, 90 };
	int		ledHueTolerance = 8;
	int		ledMinSaturation = 90;
	int		ledMinValue = 140;
	double	ledMinRadiusPx = 3.0;

	// SLAM
	int		maxFeatures = 1000;
	int		maxMapPoints = 8000;
	double	fallbackSceneDepth = 1.5;		// meters. Used to set the map scale if no LED could be used to measure it.
	double	origin[3] = { 0, 1.6, 0 };		// where the first position of the camera ends up in the world, meters
	std::string mapFile;					// loaded on start (if it exists) and saved on stop. empty = don't persist
	std::string debugImagePath;				// if not empty an annotated jpg is written here twice a second
};

struct SlamResult {
	bool	hmdTracked = false;
	double	camPos[3] = { 0, 0, 0 };		// camera optical center, world space
	bool	ledTracked[2] = { false, false };
	double	ledPos[2][3] = { {0,0,0},{0,0,0} };
	int		mapPoints = 0;
	int		trackedPoints = 0;
};

class SlamTracker {
public:
	// Returns head orientation (head -> world) as w,x,y,z. Called from the SLAM thread.
	typedef std::function<bool(double q[4])> HeadRotationProvider;
	// Called from the SLAM thread once per camera frame.
	typedef std::function<void(const SlamResult&)> ResultCallback;

	SlamTracker(const SlamConfig& config);
	~SlamTracker();

	void SetHeadRotationProvider(HeadRotationProvider p) { headRotation = p; }
	void SetResultCallback(ResultCallback c) { onResult = c; }

	bool Start();
	void Stop();
	bool IsRunning() const { return running; }

	// Forget the map and start mapping from scratch.
	void Reset() { resetRequested = true; }

private:
	struct Impl;

	void Run();

	SlamConfig cfg;
	HeadRotationProvider headRotation;
	ResultCallback onResult;
	std::thread* thread = nullptr;
	std::atomic<bool> running{ false };
	std::atomic<bool> resetRequested{ false };
	Impl* impl = nullptr;
};

#endif
