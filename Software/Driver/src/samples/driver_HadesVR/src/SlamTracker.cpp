#include "SlamTracker.h"
#include "driverlog.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <intrin.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <unordered_map>
#include <numeric>
#include <vector>

using namespace std::chrono;

namespace {

const double kPi = 3.14159265358979323846;

// ORB descriptors are 256 bit.
struct Desc256 {
	uint64_t d[4];
};

inline int Hamming(const Desc256& a, const Desc256& b)
{
	return (int)(__popcnt64(a.d[0] ^ b.d[0]) + __popcnt64(a.d[1] ^ b.d[1]) + __popcnt64(a.d[2] ^ b.d[2]) + __popcnt64(a.d[3] ^ b.d[3]));
}

Desc256 DescFromRow(const cv::Mat& m, int row)
{
	Desc256 d;
	std::memcpy(d.d, m.ptr<uint8_t>(row), sizeof(d.d));
	return d;
}

cv::Matx33d QuatToMat(double w, double x, double y, double z)
{
	const double n = std::sqrt(w * w + x * x + y * y + z * z);
	if (n < 1e-9) return cv::Matx33d::eye();
	w /= n; x /= n; y /= n; z /= n;
	return cv::Matx33d(
		1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
		2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
		2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
}

cv::Matx33d Rx(double a) { return cv::Matx33d(1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a)); }
cv::Matx33d Ry(double a) { return cv::Matx33d(cos(a), 0, sin(a), 0, 1, 0, -sin(a), 0, cos(a)); }
cv::Matx33d Rz(double a) { return cv::Matx33d(cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1); }

// closest rotation matrix to m
cv::Matx33d Orthonormalize(const cv::Matx33d& m)
{
	cv::Mat w, u, vt;
	cv::SVD::compute(cv::Mat(m), w, u, vt);
	cv::Mat r = u * vt;
	if (cv::determinant(r) < 0) {
		u.col(2) *= -1;
		r = u * vt;
	}
	return cv::Matx33d(r);
}

double Median(std::vector<double> v)
{
	if (v.empty()) return 0;
	std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
	return v[v.size() / 2];
}

struct MapPoint {
	cv::Point3d p;
	Desc256 desc;
	int seen = 0;
	int created = 0;
	int lastSeen = 0;
	bool alive = false;
};

struct Frame {
	std::vector<cv::KeyPoint> kps;
	cv::Mat descMat;
	std::vector<Desc256> desc;
	std::vector<int> mp;			// map point index per keypoint, -1 = none
	cv::Matx33d Rcw = cv::Matx33d::eye();
	cv::Vec3d tcw = cv::Vec3d(0, 0, 0);

	cv::Vec3d Center() const { return -(Rcw.t() * tcw); }
};

struct LedObservation {
	bool found = false;
	cv::Point2f px;
	float radius = 0;
	double depth = 0;				// meters along the optical axis
	cv::Vec3d camPos;				// meters, OpenCV camera axes
};

enum class State { Init, Tracking, Lost };

}  // namespace

struct SlamTracker::Impl {
	// camera
	cv::VideoCapture cap;
	cv::Matx33d K = cv::Matx33d::eye();
	cv::Mat map1, map2;
	bool undistort = false;
	cv::Size size;
	cv::Matx33d E = cv::Matx33d::eye();		// camera (openvr oriented axes) -> head
	cv::Matx33d S = cv::Matx33d(1, 0, 0, 0, -1, 0, 0, 0, -1);	// OpenCV camera axes -> openvr oriented camera axes

	cv::Ptr<cv::ORB> orb;

	// map
	std::vector<MapPoint> points;
	std::vector<int> freeSlots;
	int frameId = 0;

	// tracking
	State state = State::Init;
	std::deque<Frame> window;		// most recent keyframes, back() is the newest
	static const size_t kWindow = 8;
	Frame& LastKf() { return window.back(); }
	void PushKf(const Frame& f)
	{
		window.push_back(f);
		while (window.size() > kWindow) window.pop_front();
	}
	Frame ref;						// init reference frame
	bool haveRef = false;
	int refFrameId = 0;
	LedObservation refLed[2];
	cv::Matx33d lastRwc = cv::Matx33d::eye();
	cv::Vec3d lastCenter = cv::Vec3d(0, 0, 0);
	cv::Matx33d lastImuCam = cv::Matx33d::eye();
	bool haveLastImuCam = false;
	cv::Matx33d Rcorr = cv::Matx33d::eye();		// slam world -> openvr world
	bool corrValid = false;
	cv::Vec3d lastWorldCam = cv::Vec3d(0, 0, 0);
	int lastKfFrameId = 0;
	int trackedCount = 0;
	int lostFrames = 0;

	steady_clock::time_point lastDebugWrite;

	// metric scale estimation from the controller LEDs
	struct LedSample { cv::Vec3d Rp, C; };
	std::deque<LedSample> ledWin[2];
	int ledMissing[2] = { 0, 0 };
	bool scaleLocked = false;

	void ApplyScale(double k, Frame& cur)
	{
		for (MapPoint& m : points) if (m.alive) m.p = cv::Point3d(m.p.x * k, m.p.y * k, m.p.z * k);
		lastCenter *= k;
		for (Frame& f : window) f.tcw *= k;
		cur.tcw *= k;
		for (auto& w : ledWin) w.clear();
	}

	int AddPoint(const cv::Point3d& p, const Desc256& d)
	{
		int idx;
		if (!freeSlots.empty()) {
			idx = freeSlots.back();
			freeSlots.pop_back();
		}
		else {
			idx = (int)points.size();
			points.emplace_back();
		}
		MapPoint& m = points[idx];
		m.p = p;
		m.desc = d;
		m.seen = 1;
		m.created = frameId;
		m.lastSeen = frameId;
		m.alive = true;
		return idx;
	}

	int AlivePoints() const { return (int)points.size() - (int)freeSlots.size(); }

	void ClearMap()
	{
		points.clear();
		freeSlots.clear();
		window.clear();
		ref = Frame();
		haveRef = false;
		state = State::Init;
		corrValid = false;
		haveLastImuCam = false;
	}
};

namespace {

// Triangulates matching pixels from two views and keeps the ones that are in front of both cameras,
// reproject well and have enough parallax.
void Triangulate(const cv::Matx33d& K,
	const cv::Matx33d& R1, const cv::Vec3d& t1, const cv::Matx33d& R2, const cv::Vec3d& t2,
	const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2,
	double maxReprojPx, double minParallaxDeg,
	std::vector<cv::Point3d>& out, std::vector<char>& ok)
{
	out.assign(p1.size(), cv::Point3d());
	ok.assign(p1.size(), 0);
	if (p1.empty()) return;

	cv::Mat P1(3, 4, CV_64F), P2(3, 4, CV_64F);
	cv::Mat Rt1 = (cv::Mat_<double>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), t1[0], R1(1, 0), R1(1, 1), R1(1, 2), t1[1], R1(2, 0), R1(2, 1), R1(2, 2), t1[2]);
	cv::Mat Rt2 = (cv::Mat_<double>(3, 4) << R2(0, 0), R2(0, 1), R2(0, 2), t2[0], R2(1, 0), R2(1, 1), R2(1, 2), t2[1], R2(2, 0), R2(2, 1), R2(2, 2), t2[2]);
	P1 = cv::Mat(K) * Rt1;
	P2 = cv::Mat(K) * Rt2;

	cv::Mat a(2, (int)p1.size(), CV_64F), b(2, (int)p1.size(), CV_64F);
	for (int i = 0; i < (int)p1.size(); i++) {
		a.at<double>(0, i) = p1[i].x; a.at<double>(1, i) = p1[i].y;
		b.at<double>(0, i) = p2[i].x; b.at<double>(1, i) = p2[i].y;
	}
	cv::Mat X;
	cv::triangulatePoints(P1, P2, a, b, X);

	const cv::Vec3d c1 = -(R1.t() * t1), c2 = -(R2.t() * t2);
	const double cosMax = std::cos(minParallaxDeg * kPi / 180.0);

	for (int i = 0; i < (int)p1.size(); i++) {
		const double w = X.at<double>(3, i);
		if (std::fabs(w) < 1e-12) continue;
		const cv::Vec3d Xw(X.at<double>(0, i) / w, X.at<double>(1, i) / w, X.at<double>(2, i) / w);
		const cv::Vec3d x1 = R1 * Xw + t1, x2 = R2 * Xw + t2;
		if (x1[2] <= 0.01 || x2[2] <= 0.01) continue;

		const double e1 = std::hypot(K(0, 0) * x1[0] / x1[2] + K(0, 2) - p1[i].x, K(1, 1) * x1[1] / x1[2] + K(1, 2) - p1[i].y);
		const double e2 = std::hypot(K(0, 0) * x2[0] / x2[2] + K(0, 2) - p2[i].x, K(1, 1) * x2[1] / x2[2] + K(1, 2) - p2[i].y);
		if (e1 > maxReprojPx || e2 > maxReprojPx) continue;

		const cv::Vec3d r1 = Xw - c1, r2 = Xw - c2;
		const double cosA = r1.dot(r2) / (cv::norm(r1) * cv::norm(r2) + 1e-12);
		if (cosA > cosMax) continue;

		out[i] = cv::Point3d(Xw[0], Xw[1], Xw[2]);
		ok[i] = 1;
	}
}


cv::Matx33d Skew(const cv::Vec3d& v)
{
	return cv::Matx33d(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0);
}

cv::Matx33d ExpSO3(const cv::Vec3d& w)
{
	cv::Mat R;
	cv::Rodrigues(cv::Mat(w), R);
	return cv::Matx33d(R);
}

// Local bundle adjustment. Refines the poses of window[fixedCount..] and every map point that at least two of
// the keyframes see (the first fixedCount keyframes stay put, which anchors position and scale).
// Levenberg-Marquardt with the points eliminated through the Schur complement, Huber loss on pixel error.
void LocalBundleAdjust(std::deque<Frame>& window, std::vector<MapPoint>& points, std::vector<int>& freeSlots, const cv::Matx33d& K, int fixedCount)
{
	const int nCams = (int)window.size();
	const int nFree = nCams - fixedCount;
	if (nFree <= 0) return;

	struct Obs { int cam, pt; cv::Point2d uv; int kf, kp; };
	std::vector<Obs> obs;
	std::unordered_map<int, int> ptIndex;		// map point id -> local id
	std::vector<int> ptId;
	std::vector<int> ptCount;

	for (int c = 0; c < nCams; c++) {
		const Frame& f = window[c];
		for (int i = 0; i < (int)f.mp.size(); i++) {
			const int m = f.mp[i];
			if (m < 0 || !points[m].alive) continue;
			auto it = ptIndex.find(m);
			if (it == ptIndex.end()) {
				it = ptIndex.emplace(m, (int)ptId.size()).first;
				ptId.push_back(m);
				ptCount.push_back(0);
			}
			ptCount[it->second]++;
			obs.push_back({ c, it->second, cv::Point2d(f.kps[i].pt.x, f.kps[i].pt.y), c, i });
		}
	}
	// only points seen at least twice constrain anything
	std::vector<Obs> kept;
	for (const Obs& o : obs) if (ptCount[o.pt] >= 2) kept.push_back(o);
	obs.swap(kept);
	if (obs.size() < 30) return;

	const int nPts = (int)ptId.size();
	std::vector<cv::Matx33d> R(nCams);
	std::vector<cv::Vec3d> t(nCams), X(nPts);
	for (int c = 0; c < nCams; c++) { R[c] = window[c].Rcw; t[c] = window[c].tcw; }
	for (int j = 0; j < nPts; j++) X[j] = cv::Vec3d(points[ptId[j]].p.x, points[ptId[j]].p.y, points[ptId[j]].p.z);

	const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
	const double huber = 2.0;

	auto cost = [&](const std::vector<cv::Matx33d>& Rs, const std::vector<cv::Vec3d>& ts, const std::vector<cv::Vec3d>& Xs) {
		double total = 0;
		for (const Obs& o : obs) {
			const cv::Vec3d xc = Rs[o.cam] * Xs[o.pt] + ts[o.cam];
			if (xc[2] < 0.05) { total += 1e4; continue; }
			const double e = std::hypot(fx * xc[0] / xc[2] + cx - o.uv.x, fy * xc[1] / xc[2] + cy - o.uv.y);
			total += (e <= huber) ? e * e : 2 * huber * e - huber * huber;
		}
		return total;
	};

	double lambda = 1e-3;
	double current = cost(R, t, X);
	for (int iter = 0; iter < 8; iter++) {
		const int dim = 6 * nFree;
		cv::Mat S = cv::Mat::zeros(dim, dim, CV_64F), g = cv::Mat::zeros(dim, 1, CV_64F);
		std::vector<cv::Matx33d> Hpp(nPts, cv::Matx33d::zeros());
		std::vector<cv::Vec3d> bp(nPts, cv::Vec3d(0, 0, 0));
		struct Blk { int cam; cv::Matx<double, 6, 3> H; };
		std::vector<std::vector<Blk>> blocks(nPts);

		for (const Obs& o : obs) {
			const cv::Vec3d Rx = R[o.cam] * X[o.pt];
			const cv::Vec3d xc = Rx + t[o.cam];
			if (xc[2] < 0.05) continue;
			const double iz = 1.0 / xc[2];
			const cv::Vec2d r(fx * xc[0] * iz + cx - o.uv.x, fy * xc[1] * iz + cy - o.uv.y);
			const double e = cv::norm(r);
			const double w = (e <= huber) ? 1.0 : huber / e;

			const cv::Matx<double, 2, 3> Jp(fx * iz, 0, -fx * xc[0] * iz * iz, 0, fy * iz, -fy * xc[1] * iz * iz);
			const cv::Matx<double, 2, 3> Jx = Jp * R[o.cam];
			Hpp[o.pt] += (Jx.t() * Jx) * w;
			bp[o.pt] += cv::Vec3d((Jx.t() * r).val) * w;

			if (o.cam >= fixedCount) {
				const int k = o.cam - fixedCount;
				const cv::Matx33d nSk = -Skew(Rx);
				cv::Matx<double, 2, 6> Jc;
				const cv::Matx<double, 2, 3> Jw = Jp * nSk;
				for (int a = 0; a < 2; a++) for (int b = 0; b < 3; b++) { Jc(a, b) = Jw(a, b); Jc(a, 3 + b) = Jp(a, b); }
				const cv::Matx<double, 6, 6> Hc = (Jc.t() * Jc) * w;
				const cv::Matx<double, 6, 1> bc = (Jc.t() * r) * w;
				for (int a = 0; a < 6; a++) {
					g.at<double>(6 * k + a) += bc(a);
					for (int b = 0; b < 6; b++) S.at<double>(6 * k + a, 6 * k + b) += Hc(a, b);
				}
				blocks[o.pt].push_back({ k, (Jc.t() * Jx) * w });
			}
		}
		// damping on the camera blocks
		for (int d = 0; d < dim; d++) S.at<double>(d, d) += lambda * std::max(S.at<double>(d, d), 1e-6);

		std::vector<cv::Matx33d> Hinv(nPts);
		for (int j = 0; j < nPts; j++) {
			cv::Matx33d H = Hpp[j];
			for (int d = 0; d < 3; d++) H(d, d) += lambda * std::max(H(d, d), 1e-6);
			bool ok = true;
			Hinv[j] = H.inv(cv::DECOMP_CHOLESKY, &ok);
			if (!ok) Hinv[j] = cv::Matx33d::zeros();
			for (const Blk& a : blocks[j]) {
				const cv::Matx<double, 6, 3> HaI = a.H * Hinv[j];
				const cv::Matx<double, 6, 1> gv = HaI * bp[j];
				for (int d = 0; d < 6; d++) g.at<double>(6 * a.cam + d) -= gv(d);
				for (const Blk& b : blocks[j]) {
					const cv::Matx<double, 6, 6> sub = HaI * b.H.t();
					for (int p = 0; p < 6; p++) for (int q = 0; q < 6; q++) S.at<double>(6 * a.cam + p, 6 * b.cam + q) -= sub(p, q);
				}
			}
		}
		cv::Mat dc;
		if (!cv::solve(S, -g, dc, cv::DECOMP_CHOLESKY)) { lambda *= 10; continue; }

		std::vector<cv::Matx33d> R2 = R;
		std::vector<cv::Vec3d> t2 = t, X2 = X;
		for (int k = 0; k < nFree; k++) {
			const int c = k + fixedCount;
			const cv::Vec3d w(dc.at<double>(6 * k), dc.at<double>(6 * k + 1), dc.at<double>(6 * k + 2));
			R2[c] = ExpSO3(w) * R[c];
			t2[c] = t[c] + cv::Vec3d(dc.at<double>(6 * k + 3), dc.at<double>(6 * k + 4), dc.at<double>(6 * k + 5));
		}
		for (int j = 0; j < nPts; j++) {
			cv::Vec3d rhs = bp[j];
			for (const Blk& a : blocks[j]) {
				cv::Matx<double, 6, 1> d6;
				for (int d = 0; d < 6; d++) d6(d) = dc.at<double>(6 * a.cam + d);
				rhs += cv::Vec3d((a.H.t() * d6).val);
			}
			X2[j] = X[j] - Hinv[j] * rhs;
		}
		const double next = cost(R2, t2, X2);
		if (next < current) {
			const double gain = current - next;
			R = R2; t = t2; X = X2; current = next;
			lambda = std::max(1e-7, lambda / 3);
			if (gain < 1e-3 * current) break;
		}
		else {
			lambda *= 5;
			if (lambda > 1e6) break;
		}
	}

	// write back, and unlink observations that still don't fit
	for (int c = fixedCount; c < nCams; c++) { window[c].Rcw = R[c]; window[c].tcw = t[c]; }
	std::vector<int> bad(nPts, 0), total(nPts, 0);
	for (const Obs& o : obs) {
		const cv::Vec3d xc = R[o.cam] * X[o.pt] + t[o.cam];
		const double e = xc[2] < 0.05 ? 1e9 : std::hypot(fx * xc[0] / xc[2] + cx - o.uv.x, fy * xc[1] / xc[2] + cy - o.uv.y);
		total[o.pt]++;
		if (e > 4.0) { window[o.kf].mp[o.kp] = -1; bad[o.pt]++; }
	}
	for (int j = 0; j < nPts; j++) {
		MapPoint& m = points[ptId[j]];
		if (bad[j] * 2 > total[j]) {
			// mostly wrong: drop the point entirely, it's bad geometry
			m.alive = false;
			m.seen = 0;
			freeSlots.push_back(ptId[j]);
		}
		else {
			m.p = cv::Point3d(X[j][0], X[j][1], X[j][2]);
		}
	}
}

}  // namespace

SlamTracker::SlamTracker(const SlamConfig& config)
	: cfg(config)
{
}

SlamTracker::~SlamTracker()
{
	Stop();
	delete impl;
}

bool SlamTracker::Start()
{
	if (running) return true;
	delete impl;
	impl = new Impl();
	Impl& s = *impl;

	// Camera
	if (cfg.cameraIndex >= 0) {
		s.cap.open(cfg.cameraIndex, cv::CAP_DSHOW);
		if (!s.cap.isOpened()) {
			s.cap.open(cfg.cameraIndex);
		}
		if (!s.cap.isOpened()) {
			DriverLog("[SLAM] Could not open camera %d", cfg.cameraIndex);
			return false;
		}
		s.cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
		s.cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
		s.cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
		s.cap.set(cv::CAP_PROP_FPS, cfg.fps);
		s.cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
		if (!cfg.autoExposure) {
			s.cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);	// DirectShow: 0.25 = manual
			s.cap.set(cv::CAP_PROP_EXPOSURE, cfg.exposure);
		}

		cv::Mat first;
		for (int i = 0; i < 30 && first.empty(); i++) {
			s.cap.read(first);
		}
		if (first.empty()) {
			DriverLog("[SLAM] Camera %d opened but does not deliver frames", cfg.cameraIndex);
			s.cap.release();
			return false;
		}
		s.size = first.size();
		DriverLog("[SLAM] Camera %d running at %dx%d", cfg.cameraIndex, s.size.width, s.size.height);
	}
	else {
		s.size = cv::Size(cfg.width, cfg.height);		// frames are fed by ProcessFrame
	}

	// Intrinsics, scaled if the camera didn't give us the resolution we asked for.
	const double sx = (double)s.size.width / cfg.width, sy = (double)s.size.height / cfg.height;
	double fx = cfg.fx, fy = cfg.fy, cx = cfg.cx, cy = cfg.cy;
	if (fx <= 0) {
		fx = (s.size.width / 2.0) / std::tan(cfg.hfovDeg * kPi / 360.0);
		fy = fx;
		cx = s.size.width / 2.0;
		cy = s.size.height / 2.0;
		DriverLog("[SLAM] No camera intrinsics set, assuming a %.0f degree horizontal FOV pinhole camera. Calibrate your camera for good tracking!", cfg.hfovDeg);
	}
	else {
		fx *= sx; cx *= sx; fy *= sy; cy *= sy;
	}
	s.K = cv::Matx33d(fx, 0, cx, 0, fy, cy, 0, 0, 1);

	bool hasDist = false;
	for (double d : cfg.dist) hasDist |= (d != 0);
	if (hasDist) {
		cv::Mat D = (cv::Mat_<double>(1, 5) << cfg.dist[0], cfg.dist[1], cfg.dist[2], cfg.dist[3], cfg.dist[4]);
		cv::initUndistortRectifyMap(cv::Mat(s.K), D, cv::noArray(), cv::Mat(s.K), s.size, CV_16SC2, s.map1, s.map2);
		s.undistort = true;
	}

	s.E = Ry(cfg.camYawDeg * kPi / 180.0) * Rx(cfg.camPitchDeg * kPi / 180.0) * Rz(cfg.camRollDeg * kPi / 180.0);
	s.orb = cv::ORB::create(cfg.maxFeatures, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 15);

	// Map from a previous session
	if (!cfg.mapFile.empty()) {
		FILE* f = nullptr;
		if (fopen_s(&f, cfg.mapFile.c_str(), "rb") == 0 && f) {
			char magic[8] = { 0 };
			uint32_t n = 0;
			bool good = fread(magic, 1, 8, f) == 8 && std::memcmp(magic, "BVRMAP1\0", 8) == 0 && fread(&n, 4, 1, f) == 1 && n < 1000000;
			for (uint32_t i = 0; good && i < n; i++) {
				float p[3];
				Desc256 d;
				if (fread(p, 4, 3, f) != 3 || fread(&d, sizeof(d), 1, f) != 1) { good = false; break; }
				int idx = s.AddPoint(cv::Point3d(p[0], p[1], p[2]), d);
				s.points[idx].seen = 5;			// loaded points are trusted
			}
			fclose(f);
			if (good && s.AlivePoints() > 50) {
				s.state = State::Lost;			// relocalize against the saved map
				DriverLog("[SLAM] Loaded %d map points from %s, relocalizing...", s.AlivePoints(), cfg.mapFile.c_str());
			}
			else {
				DriverLog("[SLAM] Map file %s is invalid, starting a new map", cfg.mapFile.c_str());
				s.ClearMap();
			}
		}
		else {
			DriverLog("[SLAM] No map file at %s yet, starting a new map", cfg.mapFile.c_str());
		}
	}

	running = true;
	if (cfg.cameraIndex >= 0) thread = new std::thread(&SlamTracker::Run, this);
	return true;
}

void SlamTracker::Stop()
{
	if (running) {
		running = false;
	}
	if (thread) {
		thread->join();
		delete thread;
		thread = nullptr;
	}
	if (impl) {
		Impl& s = *impl;
		if (s.cap.isOpened()) s.cap.release();

		if (!cfg.mapFile.empty() && s.AlivePoints() >= 200 && s.state != State::Init) {
			FILE* f = nullptr;
			if (fopen_s(&f, cfg.mapFile.c_str(), "wb") == 0 && f) {
				uint32_t n = 0;
				for (const MapPoint& m : s.points) if (m.alive && m.seen >= 3) n++;
				fwrite("BVRMAP1\0", 1, 8, f);
				fwrite(&n, 4, 1, f);
				for (const MapPoint& m : s.points) {
					if (!m.alive || m.seen < 3) continue;
					float p[3] = { (float)m.p.x, (float)m.p.y, (float)m.p.z };
					fwrite(p, 4, 3, f);
					fwrite(&m.desc, sizeof(m.desc), 1, f);
				}
				fclose(f);
				DriverLog("[SLAM] Saved %u map points to %s", n, cfg.mapFile.c_str());
			}
			else {
				DriverLog("[SLAM] Could not write map file %s", cfg.mapFile.c_str());
			}
		}
	}
}

void SlamTracker::Run()
{
	Impl& s = *impl;
	cv::Mat bgr;
	int failedReads = 0;

	while (running) {
		if (!s.cap.read(bgr) || bgr.empty()) {
			if (++failedReads == 200) DriverLog("[SLAM] Camera is not delivering frames");
			std::this_thread::sleep_for(milliseconds(5));
			continue;
		}
		failedReads = 0;
		Step(bgr);
	}
}

void SlamTracker::ProcessFrame(cv::Mat& bgr)
{
	if (impl) Step(bgr);
}

void SlamTracker::Step(cv::Mat& bgr)
{
	Impl& s = *impl;
	cv::Mat hsv, gray, mask;
	const cv::Matx33d S = s.S;

	{
		if (s.undistort) {
			cv::Mat tmp;
			cv::remap(bgr, tmp, s.map1, s.map2, cv::INTER_LINEAR);
			bgr = tmp;
		}

		if (resetRequested.exchange(false)) {
			s.ClearMap();
			DriverLog("[SLAM] Map reset");
		}
		s.frameId++;

		// ---- head orientation (IMU) -> camera orientation in the world
		double q[4] = { 1, 0, 0, 0 };
		const bool haveImu = headRotation && headRotation(q);
		cv::Matx33d imuCam = cv::Matx33d::eye();		// camera (OpenCV axes) -> world, from the IMU
		if (haveImu) {
			imuCam = QuatToMat(q[0], q[1], q[2], q[3]) * s.E * S;
		}

		// ---- LEDs
		LedObservation leds[2];
		cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
		mask.create(bgr.size(), CV_8U);
		mask.setTo(255);
		for (int c = 0; c < 2; c++) {
			cv::Mat m, m2;
			const int lo = cfg.ledHue[c] - cfg.ledHueTolerance, hi = cfg.ledHue[c] + cfg.ledHueTolerance;
			const cv::Scalar smin(0, cfg.ledMinSaturation, cfg.ledMinValue), smax(0, 255, 255);
			if (lo < 0) {
				cv::inRange(hsv, cv::Scalar(0, smin[1], smin[2]), cv::Scalar(hi, 255, 255), m);
				cv::inRange(hsv, cv::Scalar(180 + lo, smin[1], smin[2]), cv::Scalar(179, 255, 255), m2);
				m |= m2;
			}
			else if (hi > 179) {
				cv::inRange(hsv, cv::Scalar(lo, smin[1], smin[2]), cv::Scalar(179, 255, 255), m);
				cv::inRange(hsv, cv::Scalar(0, smin[1], smin[2]), cv::Scalar(hi - 180, 255, 255), m2);
				m |= m2;
			}
			else {
				cv::inRange(hsv, cv::Scalar(lo, smin[1], smin[2]), cv::Scalar(hi, 255, 255), m);
			}
			cv::morphologyEx(m, m, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			cv::morphologyEx(m, m, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

			std::vector<std::vector<cv::Point>> contours;
			cv::findContours(m, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			double bestArea = 0;
			for (const auto& cn : contours) {
				const double area = cv::contourArea(cn);
				const double perim = cv::arcLength(cn, true);
				if (area < kPi * cfg.ledMinRadiusPx * cfg.ledMinRadiusPx || perim <= 0) continue;
				if (4 * kPi * area / (perim * perim) < 0.55) continue;		// not round
				{
					// an actual LED glows: it has a near saturated bright core, a poster or a shirt of the same colour doesn't
					const cv::Rect br = cv::boundingRect(cn);
					cv::Mat blobMask = cv::Mat::zeros(br.size(), CV_8U), v;
					cv::drawContours(blobMask, std::vector<std::vector<cv::Point>>{ cn }, 0, cv::Scalar(255), cv::FILLED, 8, cv::noArray(), 0, -br.tl());
					cv::extractChannel(hsv(br), v, 2);
					double peak = 0;
					cv::minMaxLoc(v, nullptr, &peak, nullptr, nullptr, blobMask);
					if (peak < cfg.ledMinPeakValue) continue;
				}
				if (area > bestArea) {
					bestArea = area;
					cv::Moments mo = cv::moments(cn);
					leds[c].found = true;
					leds[c].px = cv::Point2f((float)(mo.m10 / mo.m00), (float)(mo.m01 / mo.m00));
					leds[c].radius = (float)std::sqrt(area / kPi);
				}
			}
			if (leds[c].found) {
				// pinhole: radius_px = f * R / Z
				const double f = 0.5 * (s.K(0, 0) + s.K(1, 1));
				leds[c].depth = f * (cfg.ledDiameter * 0.5) / leds[c].radius * cfg.ledDepthScale;
				leds[c].camPos = cv::Vec3d((leds[c].px.x - s.K(0, 2)) / s.K(0, 0) * leds[c].depth,
					(leds[c].px.y - s.K(1, 2)) / s.K(1, 1) * leds[c].depth,
					leds[c].depth);
				// don't let the controllers (they move!) into the map
				cv::circle(mask, leds[c].px, (int)(leds[c].radius * 3.5f + 8), cv::Scalar(0), cv::FILLED);
			}
		}

		// ---- features
		cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
		Frame cur;
		s.orb->detectAndCompute(gray, mask, cur.kps, cur.descMat);
		cur.desc.resize(cur.kps.size());
		for (int i = 0; i < (int)cur.kps.size(); i++) cur.desc[i] = DescFromRow(cur.descMat, i);
		cur.mp.assign(cur.kps.size(), -1);

		bool poseOk = false;
		int inlierCount = 0;

		// ---- SLAM
		if (s.state == State::Init) {
			if (!s.haveRef || s.frameId - s.refFrameId > 120) {
				if (cur.kps.size() >= 200) {
					s.ref = cur;
					s.haveRef = true;
					s.refFrameId = s.frameId;
					s.refLed[0] = leds[0];
					s.refLed[1] = leds[1];
				}
			}
			else if (cur.kps.size() >= 150) {
				// match ref -> cur
				std::vector<cv::Point2f> p1, p2;
				std::vector<int> i1, i2;
				for (int j = 0; j < (int)cur.kps.size(); j++) {
					int best = 1 << 30, second = 1 << 30, bi = -1;
					for (int i = 0; i < (int)s.ref.kps.size(); i++) {
						const int d = Hamming(cur.desc[j], s.ref.desc[i]);
						if (d < best) { second = best; best = d; bi = i; }
						else if (d < second) second = d;
					}
					if (bi >= 0 && best < 60 && best < 0.75 * second) {
						p1.push_back(s.ref.kps[bi].pt);
						p2.push_back(cur.kps[j].pt);
						i1.push_back(bi);
						i2.push_back(j);
					}
				}
				std::vector<double> parallax;
				for (size_t i = 0; i < p1.size(); i++) parallax.push_back(cv::norm(p1[i] - p2[i]));

				if (p1.size() >= 100 && Median(parallax) >= 0.03 * s.size.width) {
					cv::Mat inl;
					cv::Mat Em = cv::findEssentialMat(p1, p2, cv::Mat(s.K), cv::RANSAC, 0.999, 1.0, inl);
					cv::Mat Rm, tm;
					int good = (Em.rows == 3 && Em.cols == 3) ? cv::recoverPose(Em, p1, p2, cv::Mat(s.K), Rm, tm, inl) : 0;
					if (good >= 60) {
						cv::Matx33d R(Rm);
						cv::Vec3d t(tm.at<double>(0), tm.at<double>(1), tm.at<double>(2));

						std::vector<cv::Point2f> q1, q2;
						std::vector<int> k1, k2;
						for (size_t i = 0; i < p1.size(); i++) {
							if (!inl.at<uchar>((int)i)) continue;
							q1.push_back(p1[i]); q2.push_back(p2[i]);
							k1.push_back(i1[i]); k2.push_back(i2[i]);
						}
						std::vector<cv::Point3d> X;
						std::vector<char> ok;
						Triangulate(s.K, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0), R, t, q1, q2, 2.0, 1.0, X, ok);
						int nGood = (int)std::count(ok.begin(), ok.end(), 1);

						if (nGood >= 50) {
							// ---- provisional metric scale. The translation from the essential matrix is only a direction, so
							// assume an average scene depth. The LEDs refine this as soon as the headset moves a bit.
							std::vector<double> depths;
							for (size_t i = 0; i < X.size(); i++) if (ok[i]) depths.push_back(X[i].z);
							const double scale = cfg.fallbackSceneDepth / Median(depths);
							DriverLog("[SLAM] Provisional map scale from assumed scene depth (%.2fm): %f. Hold a controller still in view and move around to measure the real scale.", cfg.fallbackSceneDepth, scale);
							s.scaleLocked = false;

							s.points.clear();
							s.freeSlots.clear();
							for (size_t i = 0; i < X.size(); i++) {
								if (!ok[i]) continue;
								const int idx = s.AddPoint(cv::Point3d(X[i].x * scale, X[i].y * scale, X[i].z * scale), cur.desc[k2[i]]);
								cur.mp[k2[i]] = idx;
							}
							cur.Rcw = R;
							cur.tcw = t * scale;
							s.window.clear();
							s.PushKf(cur);
							s.lastKfFrameId = s.frameId;
							s.haveRef = false;
							s.state = State::Tracking;
							s.corrValid = false;
							s.haveLastImuCam = false;
							s.lastRwc = R.t();
							s.lastCenter = cur.Center();
							DriverLog("[SLAM] Map initialized with %d points", s.AlivePoints());
							poseOk = true;
							inlierCount = nGood;
						}
					}
				}
			}
		}
		else {
			// predicted pose
			cv::Matx33d Rwc = s.lastRwc;
			if (haveImu && s.haveLastImuCam) {
				Rwc = s.lastRwc * (s.lastImuCam.t() * imuCam);
			}
			const cv::Vec3d center = s.lastCenter;

			std::vector<cv::Point3f> obj;
			std::vector<cv::Point2f> img;
			std::vector<int> mpIdx, kpIdx;
			cv::Mat rvec, tvec;
			std::vector<int> inliers;

			auto searchMap = [&](double radius) {
				obj.clear(); img.clear(); mpIdx.clear(); kpIdx.clear();
				const cv::Matx33d Rcw = Rwc.t();
				const cv::Vec3d tcw = -(Rcw * center);

				const int cell = 32;
				const int gw = (s.size.width + cell - 1) / cell, gh = (s.size.height + cell - 1) / cell;
				std::vector<std::vector<int>> grid(gw * gh);
				for (int i = 0; i < (int)cur.kps.size(); i++) {
					const int gx = std::min(gw - 1, std::max(0, (int)cur.kps[i].pt.x / cell));
					const int gy = std::min(gh - 1, std::max(0, (int)cur.kps[i].pt.y / cell));
					grid[gy * gw + gx].push_back(i);
				}
				std::vector<int> bestMpForKp(cur.kps.size(), -1), bestDistForKp(cur.kps.size(), 1 << 30);

				const int r = (int)std::ceil(radius / cell);
				for (int m = 0; m < (int)s.points.size(); m++) {
					const MapPoint& mpt = s.points[m];
					if (!mpt.alive) continue;
					const cv::Vec3d xc = Rcw * cv::Vec3d(mpt.p.x, mpt.p.y, mpt.p.z) + tcw;
					if (xc[2] < 0.1) continue;
					const double u = s.K(0, 0) * xc[0] / xc[2] + s.K(0, 2), v = s.K(1, 1) * xc[1] / xc[2] + s.K(1, 2);
					if (u < 0 || v < 0 || u >= s.size.width || v >= s.size.height) continue;

					const int gx = (int)u / cell, gy = (int)v / cell;
					int best = 60, bk = -1;
					for (int yy = std::max(0, gy - r); yy <= std::min(gh - 1, gy + r); yy++) {
						for (int xx = std::max(0, gx - r); xx <= std::min(gw - 1, gx + r); xx++) {
							for (int k : grid[yy * gw + xx]) {
								if (std::hypot(cur.kps[k].pt.x - u, cur.kps[k].pt.y - v) > radius) continue;
								const int d = Hamming(mpt.desc, cur.desc[k]);
								if (d < best) { best = d; bk = k; }
							}
						}
					}
					if (bk >= 0 && best < bestDistForKp[bk]) {
						bestDistForKp[bk] = best;
						bestMpForKp[bk] = m;
					}
				}
				for (int k = 0; k < (int)cur.kps.size(); k++) {
					if (bestMpForKp[k] < 0) continue;
					const MapPoint& mpt = s.points[bestMpForKp[k]];
					obj.emplace_back((float)mpt.p.x, (float)mpt.p.y, (float)mpt.p.z);
					img.push_back(cur.kps[k].pt);
					mpIdx.push_back(bestMpForKp[k]);
					kpIdx.push_back(k);
				}
			};

			auto solve = [&](bool useGuess) {
				inliers.clear();
				if (obj.size() < 12) return false;
				if (useGuess) {
					const cv::Matx33d Rcw = Rwc.t();
					const cv::Vec3d tcw = -(Rcw * center);
					cv::Rodrigues(cv::Mat(Rcw), rvec);
					tvec = (cv::Mat_<double>(3, 1) << tcw[0], tcw[1], tcw[2]);
				}
				bool ok = cv::solvePnPRansac(obj, img, cv::Mat(s.K), cv::noArray(), rvec, tvec, useGuess, 150, 3.0, 0.99, inliers,
					useGuess ? cv::SOLVEPNP_ITERATIVE : cv::SOLVEPNP_EPNP);
				if (!ok || inliers.size() < (useGuess ? 15u : 25u)) return false;
				// refine on inliers only
				std::vector<cv::Point3f> o2;
				std::vector<cv::Point2f> i2;
				for (int i : inliers) { o2.push_back(obj[i]); i2.push_back(img[i]); }
				cv::solvePnP(o2, i2, cv::Mat(s.K), cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
				return true;
			};

			bool solved = false;
			if (s.state == State::Tracking) {
				searchMap(20);
				solved = solve(true);
				if (!solved) {
					searchMap(60);
					solved = solve(true);
				}
			}
			else {
				// Lost: match against the whole map without a pose prior
				std::vector<int> alive;
				for (int m = 0; m < (int)s.points.size(); m++) if (s.points[m].alive) alive.push_back(m);
				obj.clear(); img.clear(); mpIdx.clear(); kpIdx.clear();
				for (int k = 0; k < (int)cur.kps.size(); k++) {
					int best = 1 << 30, second = 1 << 30, bm = -1;
					for (int m : alive) {
						const int d = Hamming(cur.desc[k], s.points[m].desc);
						if (d < best) { second = best; best = d; bm = m; }
						else if (d < second) second = d;
					}
					if (bm >= 0 && best < 55 && best < 0.8 * second) {
						obj.emplace_back((float)s.points[bm].p.x, (float)s.points[bm].p.y, (float)s.points[bm].p.z);
						img.push_back(cur.kps[k].pt);
						mpIdx.push_back(bm);
						kpIdx.push_back(k);
					}
				}
				solved = solve(false);
			}

			if (solved) {
				cv::Mat Rm;
				cv::Rodrigues(rvec, Rm);
				cv::Matx33d Rcw(Rm);
				cv::Vec3d tcw(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
				const cv::Vec3d newCenter = -(Rcw.t() * tcw);

				// a camera doesn't teleport
				if (s.state == State::Tracking && cv::norm(newCenter - s.lastCenter) > 1.0) {
					solved = false;
				}
				else {
					cur.Rcw = Rcw;
					cur.tcw = tcw;
					s.lastRwc = Rcw.t();
					s.lastCenter = newCenter;
					inlierCount = (int)inliers.size();
					for (int i : inliers) {
						cur.mp[kpIdx[i]] = mpIdx[i];
						MapPoint& mpt = s.points[mpIdx[i]];
						mpt.seen++;
						mpt.lastSeen = s.frameId;
					}
					poseOk = true;
					if (s.state == State::Lost) {
						DriverLog("[SLAM] Tracking (re)acquired with %d points", inlierCount);
						s.window.clear();
						s.PushKf(cur);
						s.lastKfFrameId = s.frameId;
						s.corrValid = false;
					}
					s.state = State::Tracking;
					s.lostFrames = 0;
				}
			}

			if (!solved) {
				if (s.state == State::Tracking && ++s.lostFrames > 10) {
					s.state = State::Lost;
					DriverLog("[SLAM] Tracking lost, relocalizing...");
				}
			}
			else {
				// ---- keyframes: grow the map
				std::vector<double> depths;
				const cv::Matx33d Rcw = cur.Rcw;
				for (int i : inliers) {
					const cv::Vec3d xc = Rcw * cv::Vec3d(obj[i].x, obj[i].y, obj[i].z) + cur.tcw;
					depths.push_back(xc[2]);
				}
				const double medDepth = std::max(0.3, Median(depths));
				const double dist = cv::norm(cur.Center() - s.LastKf().Center());
				const cv::Matx33d dR = s.LastKf().Rcw * cur.Rcw.t();
				const double ang = std::acos(std::min(1.0, std::max(-1.0, (dR(0, 0) + dR(1, 1) + dR(2, 2) - 1) * 0.5)));
				int kfTracked = 0;
				for (int m : s.LastKf().mp) if (m >= 0) kfTracked++;

				const bool needKf = s.frameId - s.lastKfFrameId >= 5 &&
					(dist > 0.12 * medDepth || ang > 12.0 * kPi / 180.0 || inlierCount < 0.5 * std::max(1, kfTracked));

				if (needKf) {
					// match leftover keypoints between the last keyframe and this frame
					std::vector<cv::Point2f> a, b;
					std::vector<int> ka, kb;
					for (int j = 0; j < (int)cur.kps.size(); j++) {
						if (cur.mp[j] >= 0) continue;
						int best = 1 << 30, second = 1 << 30, bi = -1;
						for (int i = 0; i < (int)s.LastKf().kps.size(); i++) {
							if (s.LastKf().mp[i] >= 0) continue;
							const int d = Hamming(cur.desc[j], s.LastKf().desc[i]);
							if (d < best) { second = best; best = d; bi = i; }
							else if (d < second) second = d;
						}
						if (bi >= 0 && best < 50 && best < 0.8 * second) {
							a.push_back(s.LastKf().kps[bi].pt);
							b.push_back(cur.kps[j].pt);
							ka.push_back(bi);
							kb.push_back(j);
						}
					}
					std::vector<cv::Point3d> X;
					std::vector<char> ok;
					Triangulate(s.K, s.LastKf().Rcw, s.LastKf().tcw, cur.Rcw, cur.tcw, a, b, 2.0, 1.5, X, ok);
					int added = 0;
					for (size_t i = 0; i < X.size(); i++) {
						if (!ok[i]) continue;
						const int idx = s.AddPoint(X[i], cur.desc[kb[i]]);
						cur.mp[kb[i]] = idx;
						s.LastKf().mp[ka[i]] = idx;
						added++;
					}
					s.PushKf(cur);
					s.lastKfFrameId = s.frameId;

					// jointly refine the recent keyframes and the points they see
					if (s.window.size() >= 3) {
						LocalBundleAdjust(s.window, s.points, s.freeSlots, s.K, 2);
						s.lastRwc = s.LastKf().Rcw.t();
						s.lastCenter = s.LastKf().Center();
					}

					// cull points that never got re-observed, and cap the map size
					if (s.AlivePoints() > cfg.maxMapPoints) {
						std::vector<int> order;
						for (int m = 0; m < (int)s.points.size(); m++) if (s.points[m].alive) order.push_back(m);
						std::sort(order.begin(), order.end(), [&](int x, int y) { return s.points[x].lastSeen < s.points[y].lastSeen; });
						const int drop = s.AlivePoints() - cfg.maxMapPoints * 9 / 10;
						for (int i = 0; i < drop; i++) { s.points[order[i]].alive = false; s.freeSlots.push_back(order[i]); }
					}
					for (int m = 0; m < (int)s.points.size(); m++) {
						MapPoint& p = s.points[m];
						if (p.alive && p.seen < 3 && s.frameId - p.created > 60) { p.alive = false; s.freeSlots.push_back(m); }
					}
					for (Frame& f : s.window) for (int& m : f.mp) if (m >= 0 && !s.points[m].alive) m = -1;
				}
			}
		}

		SlamResult res;
		res.mapPoints = s.AlivePoints();
		res.trackedPoints = inlierCount;

		// ---- metric scale from the LEDs.
		// A LED that holds still is at the same world position X in every frame:  X = R_i * p_i + g * C_i
		// (p_i = LED in camera i, measured in meters from its size; C_i = camera center in the map; g = meters per map unit).
		// So over a window of frames:  R_i p_i - mean(R p) = -g * (C_i - mean(C)), a 1D least squares for g.
		// If the LED was moving the residual is large and the estimate is thrown away.
		if (poseOk && s.state == State::Tracking) {
			for (int c = 0; c < 2; c++) {
				auto& win = s.ledWin[c];
				if (!leds[c].found) {
					if (++s.ledMissing[c] > 5) win.clear();
					continue;
				}
				s.ledMissing[c] = 0;
				win.push_back({ s.lastRwc * leds[c].camPos, s.lastCenter });
				if (win.size() > 60) win.pop_front();
				if (win.size() < 30 || s.frameId % 5 != 0) continue;

				cv::Vec3d mRp(0, 0, 0), mC(0, 0, 0);
				for (const auto& w : win) { mRp += w.Rp; mC += w.C; }
				mRp /= (double)win.size(); mC /= (double)win.size();
				double num = 0, den = 0;
				for (const auto& w : win) { num -= (w.Rp - mRp).dot(w.C - mC); den += (w.C - mC).dot(w.C - mC); }
				if (std::sqrt(den / win.size()) < 0.04) continue;			// not enough motion to see the scale
				const double g = num / den;
				double rss = 0;
				for (const auto& w : win) { const cv::Vec3d r = (w.Rp - mRp) + g * (w.C - mC); rss += r.dot(r); }
				const double rms = std::sqrt(rss / win.size());
				const double lo = s.scaleLocked ? 0.7 : 0.1, hi = s.scaleLocked ? 1.4 : 10.0;
				if (g < lo || g > hi || rms > 0.015 + 0.02 * leds[c].depth) continue;	// moving LED / bad fit

				const double k = s.scaleLocked ? std::pow(g, 0.5) : g;
				DriverLog("[SLAM] Map scale %s by controller %d: x%.3f (fit residual %.1fcm)", s.scaleLocked ? "corrected" : "measured", c, k, rms * 100);
				s.ApplyScale(k, cur);
				s.scaleLocked = true;
				break;
			}
		}

		// ---- align the map with the IMU and report
		if (poseOk) {
			const cv::Matx33d Rcw = s.lastRwc.t();
			if (haveImu) {
				if (!s.corrValid) {
					s.Rcorr = imuCam * Rcw;
					s.corrValid = true;
				}
				else {
					// slow blend towards the IMU alignment, the IMU yaw can move (recentering, drift)
					const cv::Matx33d target = imuCam * Rcw;
					s.Rcorr = Orthonormalize(s.Rcorr * 0.97 + target * 0.03);
				}
			}
			const cv::Vec3d w = s.Rcorr * s.lastCenter;
			s.lastWorldCam = cv::Vec3d(w[0] + cfg.origin[0], w[1] + cfg.origin[1], w[2] + cfg.origin[2]);
			if (haveImu) { s.lastImuCam = imuCam; s.haveLastImuCam = true; }
			res.hmdTracked = s.corrValid || !haveImu;
		}
		else if (haveImu && s.state != State::Init) {
			s.lastImuCam = imuCam;
			s.haveLastImuCam = true;
		}
		for (int k = 0; k < 3; k++) res.camPos[k] = s.lastWorldCam[k];

		// the LED sits relative to the camera, orientation from the IMU. Hold the last HMD position if SLAM is lost.
		for (int c = 0; c < 2; c++) {
			if (!leds[c].found || !haveImu) continue;
			const cv::Vec3d off = imuCam * leds[c].camPos;
			res.ledTracked[c] = true;
			for (int k = 0; k < 3; k++) res.ledPos[c][k] = s.lastWorldCam[k] + off[k];
		}

		if (onResult) onResult(res);

		// ---- debug picture
		if (!cfg.debugImagePath.empty() && duration_cast<milliseconds>(steady_clock::now() - s.lastDebugWrite).count() > 500) {
			s.lastDebugWrite = steady_clock::now();
			cv::Mat dbg = bgr.clone();
			for (const cv::KeyPoint& kp : cur.kps) cv::circle(dbg, kp.pt, 2, cv::Scalar(0, 255, 0), 1);
			for (int i = 0; i < (int)cur.kps.size(); i++) if (cur.mp[i] >= 0) cv::circle(dbg, cur.kps[i].pt, 4, cv::Scalar(0, 200, 255), 1);
			for (int c = 0; c < 2; c++) {
				if (!leds[c].found) continue;
				cv::circle(dbg, leds[c].px, (int)leds[c].radius + 4, c == 0 ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 255, 0), 2);
				char t[64];
				std::snprintf(t, sizeof(t), "%s %.2fm", c == 0 ? "R" : "L", leds[c].depth);
				cv::putText(dbg, t, leds[c].px + cv::Point2f(leds[c].radius + 6, 0), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
			}
			char t[96];
			const char* st = s.state == State::Init ? "INIT (move the headset sideways)" : (s.state == State::Tracking ? "TRACKING" : "LOST");
			std::snprintf(t, sizeof(t), "%s  map:%d  tracked:%d", st, res.mapPoints, inlierCount);
			cv::putText(dbg, t, cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
			if (dbg.cols > 960) cv::resize(dbg, dbg, cv::Size(960, dbg.rows * 960 / dbg.cols));
			cv::imwrite(cfg.debugImagePath, dbg);
		}
	}
}
