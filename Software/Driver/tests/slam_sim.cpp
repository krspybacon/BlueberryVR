// Synthetic test for SlamTracker: renders a textured box room, flies a camera through it, feeds the
// frames to the tracker with a perfect IMU orientation and compares the output to the ground truth.
//
// Build: see tests/build_slam_sim.bat

#include "SlamTracker.h"
#include "driverlog.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

const double kPi = 3.14159265358979323846;

struct Plane {
	cv::Vec3d origin, u, v, n;		// u, v unit axes, n normal
	double lenU, lenV;
	cv::Mat tex;
};

cv::Mat MakeTexture(int w, int h, unsigned seed)
{
	cv::RNG rng(seed);
	cv::Mat t(h, w, CV_8UC3);
	cv::randu(t, cv::Scalar::all(60), cv::Scalar::all(200));
	cv::GaussianBlur(t, t, cv::Size(0, 0), 2.0);
	for (int i = 0; i < w * h / 1500; i++) {
		const int g = rng.uniform(0, 255);	// muted colours like a real room, neon colours would look like the LEDs
		cv::Scalar c(g + rng.uniform(-25, 25), g + rng.uniform(-25, 25), g + rng.uniform(-25, 25));
		cv::Point p(rng.uniform(0, w), rng.uniform(0, h));
		switch (rng.uniform(0, 3)) {
		case 0: cv::circle(t, p, rng.uniform(3, 25), c, rng.uniform(-1, 3)); break;
		case 1: cv::rectangle(t, p, p + cv::Point(rng.uniform(5, 50), rng.uniform(5, 50)), c, rng.uniform(-1, 3)); break;
		default: cv::line(t, p, p + cv::Point(rng.uniform(-60, 60), rng.uniform(-60, 60)), c, 2); break;
		}
	}
	cv::GaussianBlur(t, t, cv::Size(0, 0), 0.8);
	return t;
}

// SteamVR world: +Y up, -Z forward. Room is x [-3,3], y [0,3], z [-9,3]
std::vector<Plane> MakeRoom()
{
	const double X0 = -3, X1 = 3, Y0 = 0, Y1 = 3, Z0 = -9, Z1 = 3;
	const double mpp = 0.005;		// meters per texture pixel
	std::vector<Plane> p;
	auto add = [&](cv::Vec3d o, cv::Vec3d u, cv::Vec3d v, double lu, double lv, unsigned seed) {
		Plane pl;
		pl.origin = o; pl.u = u; pl.v = v; pl.lenU = lu; pl.lenV = lv;
		pl.n = u.cross(v);
		pl.tex = MakeTexture((int)(lu / mpp), (int)(lv / mpp), seed);
		p.push_back(pl);
	};
	add({ X0, Y0, Z1 }, { 1, 0, 0 }, { 0, 0, -1 }, X1 - X0, Z1 - Z0, 1);			// floor
	add({ X0, Y1, Z1 }, { 1, 0, 0 }, { 0, 0, -1 }, X1 - X0, Z1 - Z0, 2);			// ceiling
	add({ X0, Y0, Z0 }, { 1, 0, 0 }, { 0, 1, 0 }, X1 - X0, Y1 - Y0, 3);			// front wall
	add({ X0, Y0, Z1 }, { 1, 0, 0 }, { 0, 1, 0 }, X1 - X0, Y1 - Y0, 4);			// back wall
	add({ X0, Y0, Z1 }, { 0, 0, -1 }, { 0, 1, 0 }, Z1 - Z0, Y1 - Y0, 5);			// left wall
	add({ X1, Y0, Z1 }, { 0, 0, -1 }, { 0, 1, 0 }, Z1 - Z0, Y1 - Y0, 6);			// right wall
	return p;
}

cv::Matx33d Rx(double a) { return cv::Matx33d(1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a)); }
cv::Matx33d Ry(double a) { return cv::Matx33d(cos(a), 0, sin(a), 0, 1, 0, -sin(a), 0, cos(a)); }

void MatToQuat(const cv::Matx33d& R, double q[4])
{
	const double tr = R(0, 0) + R(1, 1) + R(2, 2);
	double w, x, y, z;
	if (tr > 0) {
		const double s = std::sqrt(tr + 1) * 2;
		w = 0.25 * s; x = (R(2, 1) - R(1, 2)) / s; y = (R(0, 2) - R(2, 0)) / s; z = (R(1, 0) - R(0, 1)) / s;
	}
	else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
		const double s = std::sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
		w = (R(2, 1) - R(1, 2)) / s; x = 0.25 * s; y = (R(0, 1) + R(1, 0)) / s; z = (R(0, 2) + R(2, 0)) / s;
	}
	else if (R(1, 1) > R(2, 2)) {
		const double s = std::sqrt(1 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
		w = (R(0, 2) - R(2, 0)) / s; x = (R(0, 1) + R(1, 0)) / s; y = 0.25 * s; z = (R(1, 2) + R(2, 1)) / s;
	}
	else {
		const double s = std::sqrt(1 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
		w = (R(1, 0) - R(0, 1)) / s; x = (R(0, 2) + R(2, 0)) / s; y = (R(1, 2) + R(2, 1)) / s; z = 0.25 * s;
	}
	q[0] = w; q[1] = x; q[2] = y; q[3] = z;
}

struct Pose {
	cv::Vec3d pos;			// head position, world
	cv::Matx33d Rhead;		// head -> world
};

Pose Trajectory(double t, double speed)
{
	t *= speed;
	Pose p;
	p.pos = cv::Vec3d(0.6 * std::sin(0.4 * t), 1.6 + 0.08 * std::sin(0.7 * t), 1.5 - 0.15 * t);
	const double yaw = 40.0 * kPi / 180 * std::sin(0.3 * t), pitch = 10.0 * kPi / 180 * std::sin(0.5 * t);
	p.Rhead = Ry(yaw) * Rx(pitch);
	return p;
}

struct Camera {
	int w, h;
	double f, cx, cy, k1;		// k1: radial lens distortion

	// pixel -> ray direction (z = 1) in camera axes
	cv::Vec3d Ray(double u, double v) const
	{
		const double xd = (u - cx) / f, yd = (v - cy) / f;
		double x = xd, y = yd;
		for (int i = 0; i < 12; i++) { const double r2 = x * x + y * y; x = xd / (1 + k1 * r2); y = yd / (1 + k1 * r2); }
		return cv::Vec3d(x, y, 1);
	}
	cv::Point2d Project(const cv::Vec3d& c) const
	{
		const double x = c[0] / c[2], y = c[1] / c[2], r2 = x * x + y * y;
		return cv::Point2d(f * x * (1 + k1 * r2) + cx, f * y * (1 + k1 * r2) + cy);
	}
};

// ray cast the room
cv::Mat Render(const std::vector<Plane>& room, const Camera& cam, const cv::Matx33d& Rwc, const cv::Vec3d& C)
{
	cv::Mat img(cam.h, cam.w, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat best(cam.h, cam.w, CV_32F, cv::Scalar(1e9));
	for (const Plane& pl : room) {
		cv::Mat mx(cam.h, cam.w, CV_32F), my(cam.h, cam.w, CV_32F), tm(cam.h, cam.w, CV_32F);
		const double nOC = pl.n.dot(pl.origin - C);
		for (int y = 0; y < cam.h; y++) {
			float* px = mx.ptr<float>(y), * py = my.ptr<float>(y), * pt = tm.ptr<float>(y);
			for (int x = 0; x < cam.w; x++) {
				const cv::Vec3d d = Rwc * cam.Ray(x + 0.5, y + 0.5);
				const double nd = pl.n.dot(d);
				float tt = -1;
				if (std::fabs(nd) > 1e-9) tt = (float)(nOC / nd);
				pt[x] = tt;
				if (tt > 0) {
					const cv::Vec3d hit = C + d * (double)tt - pl.origin;
					const double a = hit.dot(pl.u), b = hit.dot(pl.v);
					px[x] = (float)(a / pl.lenU * pl.tex.cols);
					py[x] = (float)((1.0 - b / pl.lenV) * pl.tex.rows);
					if (a < 0 || a > pl.lenU || b < 0 || b > pl.lenV) pt[x] = -1;
				}
			}
		}
		cv::Mat sampled;
		cv::remap(pl.tex, sampled, mx, my, cv::INTER_LINEAR);
		cv::Mat nearer = (tm > 0) & (tm < best);
		sampled.copyTo(img, nearer);
		tm.copyTo(best, nearer);
	}
	return img;
}

}  // namespace

static int gFrame = 0;

class StdoutLog : public vr::IVRDriverLog {
public:
	virtual void Log(const char* msg) { std::printf("  [driver @%d] %s\n", gFrame, msg); }
};

int main(int argc, char** argv)
{
	const double speed = argc > 1 ? std::atof(argv[1]) : 1.0;
	const int frames = argc > 2 ? std::atoi(argv[2]) : 600;
	const bool withLed = argc > 3 ? std::atoi(argv[3]) != 0 : true;
	const double imuNoiseDeg = argc > 4 ? std::atof(argv[4]) : 0.0;
	const double pixNoise = argc > 5 ? std::atof(argv[5]) : 3.0;
	const int blackStart = argc > 6 ? std::atoi(argv[6]) : -1;		// cover the camera for blackLen frames
	const int blackLen = argc > 7 ? std::atoi(argv[7]) : 0;
	const double fps = 30.0;

	static StdoutLog logger;
	InitDriverLog(&logger);

	const int calibMode = argc > 8 ? std::atoi(argv[8]) : 1;		// 1: the tracker does not know the lens or how the camera is mounted
	const Camera cam{ 640, 480, 320.0, 323.0, 237.0, -0.15 };
	const cv::Matx33d Rmount = Ry(-8.0 * kPi / 180) * Rx(12.0 * kPi / 180);		// true camera mounting (OpenVR axes)
	std::vector<Plane> room = MakeRoom();

	SlamConfig cfg;
	cfg.cameraIndex = -1;
	cfg.width = cam.w;
	cfg.height = cam.h;
	if (calibMode == 0) {
		cfg.fx = cfg.fy = cam.f;
		cfg.cx = cam.cx;
		cfg.cy = cam.cy;
		cfg.dist[0] = cam.k1;
		cfg.camPitchDeg = 12.0;
		cfg.camYawDeg = -8.0;
	}
	else {
		cfg.hfovDeg = 75.0;		// wrong on purpose (the true value is 90)
	}
	cfg.origin[0] = cfg.origin[1] = cfg.origin[2] = 0;
	if (argc > 9) cfg.calibrationFile = argv[9];		// run twice with the same file to see a saved calibration being reused
	cfg.ledHue[0] = 150;
	cfg.fallbackSceneDepth = 3.0;

	// controller LED standing still in the world
	const cv::Vec3d ledWorld(0.35, 1.25, -0.9);
	const double ledRadius = cfg.ledDiameter / 2;
	cv::Mat hsvPix(1, 1, CV_8UC3, cv::Scalar(150, 255, 255)), bgrPix;
	cv::cvtColor(hsvPix, bgrPix, cv::COLOR_HSV2BGR);
	const cv::Scalar ledColor(bgrPix.at<cv::Vec3b>(0, 0)[0], bgrPix.at<cv::Vec3b>(0, 0)[1], bgrPix.at<cv::Vec3b>(0, 0)[2]);

	Pose current;
	cv::RNG rng(7);
	std::vector<cv::Vec3d> est, truth;
	std::vector<char> estOk;
	std::vector<cv::Vec3d> ledEst, ledTruth;
	int trackedFrames = 0, ledFrames = 0;
	double ledDepthRatioSum = 0;

	SlamTracker tracker(cfg);
	tracker.SetHeadRotationProvider([&](double q[4]) {
		cv::Matx33d R = current.Rhead;
		if (imuNoiseDeg > 0) {
			R = R * Rx(rng.gaussian(imuNoiseDeg * kPi / 180)) * Ry(rng.gaussian(imuNoiseDeg * kPi / 180));
		}
		MatToQuat(R, q);
		return true;
	});
	SlamResult last;
	tracker.SetResultCallback([&](const SlamResult& r) { last = r; });
	if (!tracker.Start()) {
		std::printf("start failed\n");
		return 1;
	}

	const cv::Matx33d S(1, 0, 0, 0, -1, 0, 0, 0, -1);
	for (int i = 0; i < frames; i++) {
		gFrame = i;
		current = Trajectory(i / fps, speed);
		const cv::Matx33d Rwc = current.Rhead * Rmount * S;
		cv::Mat img = Render(room, cam, Rwc, current.pos);

		cv::Vec3d ledCam{ 0, 0, 0 };
		if (withLed) {
			ledCam = Rwc.t() * (ledWorld - current.pos);
			if (ledCam[2] > 0.2) {
				const cv::Point2d px = cam.Project(ledCam) - cv::Point2d(0.5, 0.5);
				// sub pixel circle (4 fractional bits) so the radius isn't quantized
				const int sh = 4, one = 1 << sh;
				cv::circle(img, cv::Point((int)std::lround((px.x + 0.5) * one), (int)std::lround((px.y + 0.5) * one)),
					(int)std::lround(cam.f * ledRadius / ledCam[2] * one), ledColor, cv::FILLED, cv::LINE_AA, sh);
			}
		}
		if (pixNoise > 0) {
			cv::Mat noise(img.size(), CV_16SC3);
			cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(pixNoise));
			cv::Mat tmp;
			img.convertTo(tmp, CV_16SC3);
			tmp += noise;
			tmp.convertTo(img, CV_8UC3);
		}

		if (blackStart >= 0 && i >= blackStart && i < blackStart + blackLen) img.setTo(cv::Scalar(0, 0, 0));
		last = SlamResult();
		tracker.ProcessFrame(img);

		if (last.hmdTracked) {
			trackedFrames++;
			est.push_back(cv::Vec3d(last.camPos[0], last.camPos[1], last.camPos[2]));
			truth.push_back(current.pos);
			estOk.push_back(1);
		}
		if (last.ledTracked[0]) {
			ledFrames++;
			ledEst.push_back(cv::Vec3d(last.ledPos[0][0], last.ledPos[0][1], last.ledPos[0][2]));
			ledTruth.push_back(ledWorld);
			const cv::Vec3d off(last.ledPos[0][0] - last.camPos[0], last.ledPos[0][1] - last.camPos[1], last.ledPos[0][2] - last.camPos[2]);
			ledDepthRatioSum += cv::norm(off) / cv::norm(ledWorld - current.pos);
		}
		if (i % 100 == 99) {
			std::printf("frame %d: tracked=%d map=%d inliers=%d\n", i + 1, (int)last.hmdTracked, last.mapPoints, last.trackedPoints);
			if (i == 99) cv::imwrite("sim_frame100.jpg", img);
		}
	}

	// ---- metrics. Position is relative to the start of the map, so align on translation and fit scale
	std::printf("\nframes tracked: %d / %d, LED seen in %d\n", trackedFrames, frames, ledFrames);
	auto report = [&](const char* name, size_t from) {
		if (est.size() < from + 10) return;
		const size_t n = est.size() - from;
		cv::Vec3d me(0, 0, 0), mt(0, 0, 0);
		for (size_t i = from; i < est.size(); i++) { me += est[i]; mt += truth[i]; }
		me /= (double)n; mt /= (double)n;
		double num = 0, den = 0, se = 0, maxe = 0, pathLen = 0;
		for (size_t i = from; i < est.size(); i++) {
			num += (est[i] - me).dot(truth[i] - mt); den += (est[i] - me).dot(est[i] - me);
			const double e = cv::norm((est[i] - me) - (truth[i] - mt));
			se += e * e; maxe = std::max(maxe, e);
			if (i > from) pathLen += cv::norm(truth[i] - truth[i - 1]);
		}
		std::printf("%-22s RMSE %.3f m (max %.3f) over %.1f m of travel, scale error %+.1f%%\n", name, std::sqrt(se / n), maxe, pathLen, (den / num - 1) * 100);
	};
	report("whole run:", 0);
	report("steady state (last half):", est.size() / 2);
	if (!ledEst.empty()) {
		// LED is reported as camera + offset so shares the map's frame; compare LED relative to the camera motion
		cv::Vec3d m(0, 0, 0);
		for (auto& e : ledEst) m += e;
		m /= (double)ledEst.size();
		double sd = 0;
		for (auto& e : ledEst) sd += (e - m).dot(e - m);
		std::printf("LED world position jitter (std dev, it is standing still): %.3f m\n", std::sqrt(sd / ledEst.size()));
	}
	const SlamCalibration cal = tracker.GetCalibration();
	std::printf("learned lens: f=%.1f (true %.1f) cx=%.1f (%.1f) cy=%.1f (%.1f) k1=%.3f (%.3f)\n", cal.f, cam.f, cal.cx, cam.cx, cal.cy, cam.cy, cal.k1, cam.k1);
	{
		const cv::Matx33d Rl(cal.mount[0], cal.mount[1], cal.mount[2], cal.mount[3], cal.mount[4], cal.mount[5], cal.mount[6], cal.mount[7], cal.mount[8]);
		const cv::Matx33d D = Rl.t() * (Rmount * S);
		const double ang = std::acos(std::min(1.0, std::max(-1.0, (D(0, 0) + D(1, 1) + D(2, 2) - 1) * 0.5)));
		std::printf("mount rotation error: %.2f degrees (lens locked: %d, mount calibrated: %d)\n", ang * 180 / kPi, (int)cal.intrinsicsLocked, (int)cal.mountCalibrated);
	}
	if (ledFrames) std::printf("LED distance measured / true = %.3f\n", ledDepthRatioSum / ledFrames);
	tracker.Stop();
	return 0;
}
