/* changed by liu to fit opencv4 and extended to overlay LiDAR-based people detector results */

#include <vector>
#include <string>
#include <iostream>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <limits>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"

#include <opencv2/opencv.hpp>

#include "MyEllipseNormalEvaluation_2LS.h"
#include "MyCondensation.h"
#include "json11.hpp"

// =========================
// Constants
// =========================
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

// =========================
// Global mouse input helper
// =========================
bool lbPressed = false;
int lbX = 0;
int lbY = 0;

void mouseCallback(int event, int x, int y, int, void*)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		lbPressed = true;
		lbX = x;
		lbY = y;
	}
}

// =========================
// Visualization parameters
// =========================
int isTracked = 0;
int aveIntensity = 0;

float Scale = 0.075f;  // pixel = millimeter * Scale
cv::Point windowSize;

float ThresL = 3000;	// mm
float ThresM = 1000;	// mm
float ThresS = 100;		// mm

// =========================
// Laser Scan handler
// =========================
class ScanMessageHandler
{
protected:
	static constexpr int DEF_SCAN_STEP_MAX = 1080;
	static constexpr double DEF_ANGLE_OFFSET = M_PI;

public:
	static constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;  // m -> mm
	static constexpr int INVALID_DISTANCE = -1;
	static double RetroreflectiveIntensity;

	std::string topicName;
	double offsetX = 0.0;
	double offsetY = 0.0;
	double offsetT = 0.0;

	std::vector<int> urgDistance;
	std::vector<int> urgIntensity;
	std::vector<double> sinVal;
	std::vector<double> cosVal;
	std::vector<cv::Point2d> drawPosition;

	void TopicCallbackFunction(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		const std::size_t n = msg->ranges.size();
		if (static_cast<int>(n) != dataSize)
		{
			urgDistance.resize(n);
			urgIntensity.resize(n);
			CreateTrigonometricTable(msg->angle_min, msg->angle_increment, static_cast<int>(n));
			dataSize = static_cast<int>(n);
		}

		const std::size_t nIntens = msg->intensities.size();
		for (std::size_t i = 0; i < n; ++i)
		{
			const double range = msg->ranges[i];
			if (!std::isfinite(range) || range <= 0.0)
			{
				urgDistance[i] = INVALID_DISTANCE;
				urgIntensity[i] = 0;
				continue;
			}

			double dist_mm = range * SCAN_DISTANCE_MAGNIFICATION;
			if (!std::isfinite(dist_mm) || dist_mm > static_cast<double>(std::numeric_limits<int>::max()))
			{
				dist_mm = static_cast<double>(std::numeric_limits<int>::max());
			}
			urgDistance[i] = static_cast<int>(std::round(dist_mm));

			double intensityValue = 0.0;
			if (i < nIntens && std::isfinite(msg->intensities[i]))
			{
				intensityValue = std::max(0.0, static_cast<double>(msg->intensities[i]));
			}
			urgIntensity[i] = static_cast<int>(intensityValue);
		}

		IntensityNormalization();
		CalcDrawPosition();
	}

	void SetOffset(double x, double y, double th)
	{
		offsetX = x;
		offsetY = y;
		offsetT = th;
		dataSize = 0;
	}

protected:
	void CreateTrigonometricTable(double angle_min, double angle_increment, int num)
	{
		sinVal.resize(num);
		cosVal.resize(num);
		for (int i = 0; i < num; ++i)
		{
			const double ang = angle_min + angle_increment * i + offsetT + DEF_ANGLE_OFFSET;
			sinVal[i] = std::sin(ang);
			cosVal[i] = std::cos(ang);
		}
	}

	void IntensityNormalization()
	{
		const int limit = std::min(static_cast<int>(urgDistance.size()), static_cast<int>(urgIntensity.size()));
		for (int i = 0; i < limit; ++i)
		{
			if (urgDistance[i] <= INVALID_DISTANCE) continue;

			const double normalized = urgIntensity[i] * std::pow(static_cast<double>(urgDistance[i]), 0.5);
			int intensity = static_cast<int>(std::round(normalized / RetroreflectiveIntensity * 255.0));
			intensity = std::max(0, std::min(255, intensity));
			urgIntensity[i] = intensity;
		}
	}

	void CalcDrawPosition()
	{
		if (drawPosition.size() != static_cast<std::size_t>(dataSize))
		{
			drawPosition.resize(dataSize);
		}

		const int limit = std::min(static_cast<int>(urgDistance.size()), dataSize);
		for (int i = 0; i < limit; ++i)
		{
			if (urgDistance[i] <= INVALID_DISTANCE) continue;
			if (urgDistance[i] > ThresS && urgDistance[i] < ThresL)
			{
				drawPosition[i].x = std::round(urgDistance[i] * Scale * sinVal[i]) + windowSize.x * 0.5 - offsetY * Scale;
				drawPosition[i].y = std::round(urgDistance[i] * Scale * cosVal[i]) + windowSize.y * 0.5 - offsetX * Scale;
			}
			else
			{
				drawPosition[i].x = std::numeric_limits<int>::min();
				drawPosition[i].y = std::numeric_limits<int>::min();
			}
		}
	}

private:
	int dataSize = 0;
};

double ScanMessageHandler::RetroreflectiveIntensity = 200000.0;

// =========================
// Helper functions
// =========================
static double quaternionToYaw(const geometry_msgs::Quaternion& q)
{
	const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

// Map PoseArray pose (meters) -> display pixel point.
// Returns false if mapping fails.
static bool poseToDisplayPoint(const geometry_msgs::Pose& pose,
                               double outputOffsetX,
                               double outputOffsetY,
                               double outputOffsetYaw,
                               cv::Point& dst)
{
	const double diffX = pose.position.x - outputOffsetX;
	const double diffY = pose.position.y - outputOffsetY;

	if (!std::isfinite(diffX) || !std::isfinite(diffY))
	{
		return false;
	}

	const double mmScale = ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale;
	const double rotX = -diffX * mmScale;
	const double rotY =  diffY * mmScale;

	const double cosT = std::cos(outputOffsetYaw);
	const double sinT = std::sin(outputOffsetYaw);

	const double pixelX = rotX * cosT + rotY * sinT;
	const double pixelY = -rotX * sinT + rotY * cosT;

	const double px = pixelX + windowSize.x * 0.5;
	const double py = pixelY + windowSize.y * 0.5;

	if (!std::isfinite(px) || !std::isfinite(py))
	{
		return false;
	}

	dst.x = static_cast<int>(std::round(px));
	dst.y = static_cast<int>(std::round(py));
	return true;
}

// =========================
// Main
// =========================
int main(int argc, char** argv)
{
	using namespace cv;

	// --- Default configuration ---
	std::string nodeName = "person_following_multi";
	std::string topicNameTrackingPosition = "pose_person_following";
	double outputOffsetPoseX = 0.0;
	double outputOffsetPoseY = 0.0;
	double outputOffsetPoseT = -0.5 * M_PI;

	double startTrackingIntensityMin = 80.0;
	double startTrackingDistanceMax  = 2200.0;
	double stopTrackingIntensity     = 32.0;
	ScanMessageHandler::RetroreflectiveIntensity = 200000.0;

	std::string peopleDetectorPoseTopic = "/people/poses";
	bool enablePeopleOverlay  = true;
	bool enablePeopleSeeding  = true;

	windowSize = Point(640, 640);

	// --- Prepare handlers ---
	std::vector<ScanMessageHandler> handlerList;
	handlerList.emplace_back();
	handlerList.back().SetOffset(0, 0, 0);
	handlerList.back().topicName = "scan";

	int offsetX[2] = { windowSize.x / 2, windowSize.x / 2 };
	int offsetY[2] = { windowSize.y / 2, windowSize.y / 2 };
	double offsetT[2] = { 0.0, 0.0 };

	// --- Load optional JSON configuration ---
	if (argc >= 2)
	{
		auto configure = json11::LoadJsonFile(argv[1]);
		if (configure == nullptr)
		{
			ROS_ERROR_STREAM("Cannot read configure file: " << argv[1]);
		}
		else
		{
			nodeName                  = configure["NodeName"].is_null() ? nodeName : configure["NodeName"].string_value();
			startTrackingIntensityMin = configure["StartTrackingIntensityMin"].is_null() ? startTrackingIntensityMin : configure["StartTrackingIntensityMin"].number_value();
			startTrackingDistanceMax  = configure["StartTrackingDistanceMax"].is_null() ? startTrackingDistanceMax : configure["StartTrackingDistanceMax"].number_value();
			stopTrackingIntensity     = configure["StopTrackingIntensity"].is_null() ? stopTrackingIntensity : configure["StopTrackingIntensity"].number_value();
			topicNameTrackingPosition = configure["TopicNamePublishPosition"].is_null() ? topicNameTrackingPosition : configure["TopicNamePublishPosition"].string_value();
			ScanMessageHandler::RetroreflectiveIntensity =
				configure["RetroreflectiveIntensity"].is_null() ? ScanMessageHandler::RetroreflectiveIntensity : configure["RetroreflectiveIntensity"].number_value();

			windowSize.x = configure["ImageSize"]["x"].is_null() ? windowSize.x : configure["ImageSize"]["x"].number_value();
			windowSize.y = configure["ImageSize"]["y"].is_null() ? windowSize.y : configure["ImageSize"]["y"].number_value();

			outputOffsetPoseX = configure["OutputOffset"]["x"].is_null() ? outputOffsetPoseX : configure["OutputOffset"]["x"].number_value();
			outputOffsetPoseY = configure["OutputOffset"]["y"].is_null() ? outputOffsetPoseY : configure["OutputOffset"]["y"].number_value();
			outputOffsetPoseT = configure["OutputOffset"]["theta"].is_null() ? outputOffsetPoseT : configure["OutputOffset"]["theta"].number_value();

			ThresS = configure["ThresholdDistance"]["short"].is_null() ? ThresS : configure["ThresholdDistance"]["short"].number_value();
			ThresM = configure["ThresholdDistance"]["medium"].is_null() ? ThresM : configure["ThresholdDistance"]["medium"].number_value();
			ThresL = configure["ThresholdDistance"]["long"].is_null() ? ThresL : configure["ThresholdDistance"]["long"].number_value();

			if (configure["ScanList"].is_array())
			{
				auto slist = configure["ScanList"].array_items();
				if (!slist.empty())
				{
					handlerList.clear();
					int idx = 0;
					for (const auto& scanCfg : slist)
					{
						if (scanCfg["Offset"]["x"].is_null() || scanCfg["Offset"]["y"].is_null() ||
						    scanCfg["Offset"]["theta"].is_null() || scanCfg["TopicName"].is_null())
						{
							continue;
						}
						if (idx >= 2) break;

						offsetX[idx] = -scanCfg["Offset"]["y"].number_value() * Scale + windowSize.x / 2;
						offsetY[idx] = -scanCfg["Offset"]["x"].number_value() * Scale + windowSize.y / 2;
						offsetT[idx] = -scanCfg["Offset"]["theta"].number_value();

						handlerList.emplace_back();
						handlerList.back().SetOffset(scanCfg["Offset"]["x"].number_value(),
						                             scanCfg["Offset"]["y"].number_value(),
						                             scanCfg["Offset"]["theta"].number_value());
						handlerList.back().topicName = scanCfg["TopicName"].string_value();
						++idx;
					}
				}
			}

			if (!configure["PeopleDetector"].is_null())
			{
				const auto pd = configure["PeopleDetector"];
				if (!pd["pose_topic"].is_null())
				{
					peopleDetectorPoseTopic = pd["pose_topic"].string_value();
				}
				if (!pd["enable_overlay"].is_null())
				{
					enablePeopleOverlay = pd["enable_overlay"].bool_value();
				}
				if (!pd["enable_seeding"].is_null())
				{
					enablePeopleSeeding = pd["enable_seeding"].bool_value();
				}
			}
		}
	}

	const int centerX = windowSize.x / 2;
	const int centerY = windowSize.y / 2;

	// --- Sensor visible mask (binary visibility per sensor) ---
	cv::Mat sensorVisibleRange[2] = {
		cv::Mat(windowSize.y, windowSize.x, CV_8UC1, cv::Scalar(0)),
		cv::Mat(windowSize.y, windowSize.x, CV_8UC1, cv::Scalar(0))
	};
	for (int i = 0; i < 2; ++i)
	{
		const double oft = offsetT[i] + M_PI * 0.5;
		cv::Vec2d ofv(std::cos(oft), std::sin(oft));
		const double costh = std::cos(0.25 * M_PI);
		for (int r = 0; r < sensorVisibleRange[i].rows; ++r)
		{
			for (int c = 0; c < sensorVisibleRange[i].cols; ++c)
			{
				cv::Vec2d pv(c - offsetX[i], r - offsetY[i]);
				const double normpv = cv::norm(pv);
				if (normpv < 1e-6)
				{
					sensorVisibleRange[i].at<uchar>(r, c) = 0;
					continue;
				}
				const double co = pv.dot(ofv) / normpv;
				sensorVisibleRange[i].at<uchar>(r, c) = (co < costh) ? 255 : 0;
			}
		}
	}
	{
		cv::Mat tempNot;
		cv::bitwise_not(sensorVisibleRange[1], tempNot);
		cv::bitwise_and(sensorVisibleRange[0], tempNot, sensorVisibleRange[0]);
	}

	// --- ROS setup ---
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<std::string>("people_detector_pose_topic", peopleDetectorPoseTopic, peopleDetectorPoseTopic);
	pnh.param<bool>("people_detector_enable_overlay", enablePeopleOverlay, enablePeopleOverlay);
	pnh.param<bool>("people_detector_enable_seeding", enablePeopleSeeding, enablePeopleSeeding);

	std::vector<ros::Subscriber> laserSubs;
	laserSubs.reserve(handlerList.size());
	for (auto& handler : handlerList)
	{
		laserSubs.push_back(nh.subscribe(handler.topicName.c_str(), 100, &ScanMessageHandler::TopicCallbackFunction, &handler));
	}

	ros::Publisher pubPosePersonFollowing = nh.advertise<geometry_msgs::Pose2D>(topicNameTrackingPosition, 100);
	geometry_msgs::Pose2D msgPosePersonFollowing;

	std::vector<geometry_msgs::Pose> peopleDetections;
	std::mutex peopleDetectionsMutex;
	std::string peopleDetectionsFrameId;

	ros::Subscriber peopleSub;
	if (enablePeopleOverlay && !peopleDetectorPoseTopic.empty())
	{
		auto callback = [&](const geometry_msgs::PoseArray::ConstPtr& msg)
		{
			std::lock_guard<std::mutex> lock(peopleDetectionsMutex);
			peopleDetections = msg->poses;
			peopleDetectionsFrameId = msg->header.frame_id;
		};
		peopleSub = nh.subscribe<geometry_msgs::PoseArray>(peopleDetectorPoseTopic, 1, callback);
		ROS_INFO_STREAM("person_following_multiple_OR: subscribed to " << peopleDetectorPoseTopic);
	}
	else if (!peopleDetectorPoseTopic.empty())
	{
		ROS_WARN_STREAM("person_following_multiple_OR: overlay disabled, not subscribing to PoseArray");
	}

	// --- Visualization buffers ---
	cv::Size imageSize(windowSize.x, windowSize.y);
	cv::Mat baseImage(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat distanceImage(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat intensityImage(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat distanceImage2(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat intensityImage2(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat dispImage(imageSize, CV_8UC3);
	cv::Mat dispImage2(imageSize, CV_8UC3);
	cv::Mat distImage(imageSize, CV_8UC1);
	cv::Mat distImageNot(imageSize, CV_8UC1);
	cv::Mat transImage(imageSize, CV_32F);

	// Draw reference rings
	constexpr int SCAN_STEP_NUM = 1080;
	constexpr float Rotate = 45.0f;
	cv::circle(baseImage, cv::Point(centerX, centerY), 4, cv::Scalar(64, 64, 64), -1, cv::LINE_AA);
	for (int jj = 0; jj < SCAN_STEP_NUM; ++jj)
	{
		double angle = (jj * 0.25 + Rotate) * (M_PI / 180.0);
		auto drawCircle = [&](float radius, int shade)
		{
			cv::circle(baseImage,
			           cv::Point(std::round(radius * Scale * std::sin(angle)) + centerX,
			                     std::round(radius * Scale * std::cos(angle)) + centerY),
			           1,
			           cv::Scalar(shade, shade, shade),
			           -1,
			           cv::LINE_AA);
		};
		drawCircle(ThresS, 64);
		drawCircle(ThresM, 32);
		drawCircle(ThresL, 64);
	}

	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("Display Image", mouseCallback);

	// --- Condensation tracker ---
	MyCondensation* ConDens = myCreateConDensation(3, 300);
	cv::Mat initValue = cv::Mat::zeros(3, 1, CV_32F);
	cv::Mat initMean = cv::Mat::zeros(3, 1, CV_32F);
	cv::Mat initDeviation = cv::Mat::zeros(3, 1, CV_32F);

	initValue.at<float>(2) = 1080.0f;
	initDeviation.at<float>(0) = 5.0f;
	initDeviation.at<float>(1) = 5.0f;
	initDeviation.at<float>(2) = 20.0f;

	myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
	myConDensUpdateSample(ConDens);

	SetSensorPosition_2LS(0, cv::Point(offsetX[0], offsetY[0]));
	SetSensorPosition_2LS(1, cv::Point(offsetX[1], offsetY[1]));
	SetSensorVisibleRange_2LS(0, sensorVisibleRange[0]);
	SetSensorVisibleRange_2LS(1, sensorVisibleRange[1]);
	StoreBodyContourPosition_2LS(18, 11, 10);
	StoreHeadContourPosition_2LS(9, 10);

	int prevUsrU = -1;
	int prevUsrV = -1;
	int prevUsrAng = -1;

	auto seedCondensation = [&](int pixelX, int pixelY, int angleDeg)
	{
		initValue.at<float>(0) = static_cast<float>(pixelX);
		initValue.at<float>(1) = static_cast<float>(pixelY);
		initValue.at<float>(2) = static_cast<float>(angleDeg);
		myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
		myConDensUpdateSample(ConDens);
		myConDensUpdateByTime(ConDens);
		isTracked = 1;
	};

	ros::Rate loopRate(60);
	while (ros::ok())
	{
		baseImage.copyTo(dispImage);
		baseImage.copyTo(distanceImage);
		baseImage.copyTo(dispImage2);
		distanceImage2.setTo(cv::Scalar(0, 0, 0));
		distanceImage.setTo(cv::Scalar(0, 0, 0));
		intensityImage.setTo(cv::Scalar(0, 0, 0));
		intensityImage2.setTo(cv::Scalar(0, 0, 0));

		std::vector<geometry_msgs::Pose> localDetections;
		{
			std::lock_guard<std::mutex> lock(peopleDetectionsMutex);
			localDetections = peopleDetections;
		}

		int loopNumber = 1;
		for (const auto& handler : handlerList)
		{
			const std::size_t n = handler.drawPosition.size();
			for (std::size_t i = 0; i < n; ++i)
			{
				if (handler.urgDistance[i] <= ScanMessageHandler::INVALID_DISTANCE) continue;

				cv::Scalar pointColor(handler.urgIntensity[i] * 4 + 64,
				                      handler.urgIntensity[i] * 4 + 64,
				                      64);
				pointColor[0] = std::min(255., pointColor[0]);
				pointColor[1] = std::min(255., pointColor[1]);

				const auto& px = handler.drawPosition[i];
				if (px.x == std::numeric_limits<int>::min() || px.y == std::numeric_limits<int>::min()) continue;

				if (loopNumber == 1)
				{
					cv::circle(distanceImage, px, 1, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
					cv::circle(intensityImage, px, 1,
					           cv::Scalar(handler.urgIntensity[i], handler.urgIntensity[i], handler.urgIntensity[i]), -1, cv::LINE_AA);
					cv::circle(dispImage, px, 1, pointColor, -1, cv::LINE_AA);
				}
				else if (loopNumber == 2)
				{
					cv::circle(distanceImage2, px, 1, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
					cv::circle(intensityImage2, px, 1,
					           cv::Scalar(handler.urgIntensity[i], handler.urgIntensity[i], handler.urgIntensity[i]), -1, cv::LINE_AA);
					cv::circle(dispImage2, px, 1, pointColor, -1, cv::LINE_AA);
				}
			}
			++loopNumber;
		}

		cv::add(distanceImage, distanceImage2, distanceImage);
		cv::add(intensityImage, intensityImage2, intensityImage);
		cv::add(dispImage, dispImage2, dispImage);

		if (enablePeopleOverlay && !localDetections.empty())
		{
			int label = 0;
			for (const auto& pose : localDetections)
			{
				cv::Point pt;
				if (!poseToDisplayPoint(pose, outputOffsetPoseX, outputOffsetPoseY, outputOffsetPoseT, pt)) continue;
				if (pt.x < 0 || pt.x >= windowSize.x || pt.y < 0 || pt.y >= windowSize.y) continue;

				cv::circle(dispImage, pt, 6, cv::Scalar(0, 200, 0), 2, cv::LINE_AA);

				const double yaw = quaternionToYaw(pose.orientation);
				const double arrowLen = 20.0;
				cv::Point arrowTip(pt.x + static_cast<int>(std::cos(yaw + outputOffsetPoseT) * arrowLen),
				                   pt.y - static_cast<int>(std::sin(yaw + outputOffsetPoseT) * arrowLen));
				cv::arrowedLine(dispImage, pt, arrowTip, cv::Scalar(0, 220, 0), 2, cv::LINE_AA, 0, 0.25);

				cv::putText(dispImage,
				            std::to_string(label++),
				            pt + cv::Point(8, -6),
				            cv::FONT_HERSHEY_PLAIN,
				            1.0,
				            cv::Scalar(0, 220, 0),
				            1);
			}
		}

		cv::imshow("Intensity Image", intensityImage);

		cv::cvtColor(distanceImage, distImage, cv::COLOR_BGR2GRAY);
		cv::threshold(distImage, distImageNot, 64.0, 255.0, cv::THRESH_BINARY_INV);
		cv::distanceTransform(distImageNot, transImage, cv::DIST_L2, 3);

		myConDensUpdateSample(ConDens);
		for (int i = 0; i < ConDens->samples_num; ++i)
		{
			const int sx = std::round(ConDens->samples[i].at<float>(0));
			const int sy = std::round(ConDens->samples[i].at<float>(1));
			const int sa = std::round(ConDens->samples[i].at<float>(2));
			ConDens->confidence.at<float>(0, i) = CalculateBodyLikelihood_2LS(transImage, cv::Point(sx, sy), sa);
		}
		myConDensUpdateByTime(ConDens);

		int usrU = std::round(ConDens->state.at<float>(0));
		int usrV = std::round(ConDens->state.at<float>(1));
		int usrAng = std::round(ConDens->state.at<float>(2));
		usrAng = (usrAng % 360 + 360) % 360;

		cv::cvtColor(intensityImage, distImage, cv::COLOR_BGR2GRAY);
		cv::Rect roiRect(usrU - 30, usrV - 30, 60, 60);
		roiRect &= cv::Rect(0, 0, distImage.cols, distImage.rows);
		if (roiRect.area() > 0)
		{
			const cv::Mat roi = distImage(roiRect);
			const int pixelCount = cv::countNonZero(roi);
			aveIntensity = (pixelCount > 0) ? static_cast<int>(cv::sum(roi)[0] / pixelCount) : 0;
		}
		else
		{
			aveIntensity = 0;
		}

		if (isTracked == 1 && aveIntensity < stopTrackingIntensity)
		{
			initValue.at<float>(0) = 0.0f;
			initValue.at<float>(1) = 0.0f;
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
			myConDensUpdateSample(ConDens);
			myConDensUpdateByTime(ConDens);
			isTracked = 0;
		}

		bool seededByDetector = false;
		if (isTracked == 0 && enablePeopleSeeding && !localDetections.empty())
		{
			for (const auto& pose : localDetections)
			{
				cv::Point pt;
				if (!poseToDisplayPoint(pose, outputOffsetPoseX, outputOffsetPoseY, outputOffsetPoseT, pt)) continue;
				if (pt.x < 0 || pt.x >= windowSize.x || pt.y < 0 || pt.y >= windowSize.y) continue;

				const double yawDeg = quaternionToYaw(pose.orientation) * RAD_TO_DEG;
				seedCondensation(pt.x, pt.y, static_cast<int>(std::round(yawDeg)));
				seededByDetector = true;
				break;
			}
		}

		if (isTracked == 0 && !seededByDetector)
		{
			for (const auto& handler : handlerList)
			{
				for (std::size_t i = 0; i < handler.urgDistance.size(); ++i)
				{
					if (handler.urgDistance[i] <= ScanMessageHandler::INVALID_DISTANCE) continue;
					if (handler.urgDistance[i] < startTrackingDistanceMax && handler.urgIntensity[i] > startTrackingIntensityMin)
					{
						const int seedX = static_cast<int>(std::round(handler.drawPosition[i].x));
						const int seedY = static_cast<int>(std::round(handler.drawPosition[i].y));
						if (seedX == std::numeric_limits<int>::min() || seedY == std::numeric_limits<int>::min()) continue;

						seedCondensation(seedX, seedY, (prevUsrAng >= 0) ? prevUsrAng : 0);
						break;
					}
				}
				if (isTracked == 1) break;
			}
		}

		if (lbPressed)
		{
			lbPressed = false;
			seedCondensation(lbX, lbY, 0);
		}

		if (!ConDens->state.empty())
		{
			usrU = std::round(ConDens->state.at<float>(0));
			usrV = std::round(ConDens->state.at<float>(1));
			usrAng = std::round(ConDens->state.at<float>(2));

			cv::ellipse(dispImage, cv::Point(usrU, usrV), cv::Size(24, 12), usrAng, 0, 360, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
			cv::rectangle(dispImage, cv::Point(usrU - 30, usrV - 30), cv::Point(usrU + 30, usrV + 30), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
			char text[32];
			std::snprintf(text, sizeof(text), "%4d", aveIntensity);
			cv::putText(dispImage, text, cv::Point(usrU + 30, usrV + 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255), 1);
			std::snprintf(text, sizeof(text), "Ang:%3d", usrAng);
			cv::putText(dispImage, text, cv::Point(usrU + 30, usrV + 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255), 1);
		}

		if (!ConDens->state.empty())
		{
			const double rotX = (usrU - centerX) * std::cos(outputOffsetPoseT) - (usrV - centerY) * std::sin(outputOffsetPoseT);
			const double rotY = (usrU - centerX) * std::sin(outputOffsetPoseT) + (usrV - centerY) * std::cos(outputOffsetPoseT);

			msgPosePersonFollowing.x = outputOffsetPoseX + (-rotX) / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.y = outputOffsetPoseY - (-rotY) / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.theta = outputOffsetPoseT + (-usrAng * DEG_TO_RAD + M_PI * 1.5);
			pubPosePersonFollowing.publish(msgPosePersonFollowing);

			prevUsrU = usrU;
			prevUsrV = usrV;
			prevUsrAng = usrAng;
		}

		cv::imshow("Display Image", dispImage);
		if (cv::waitKey(1) == 'q') break;

		ros::spinOnce();
		loopRate.sleep();
	}

	myReleaseConDensation(&ConDens);
	cv::destroyAllWindows();
	return 0;
}
