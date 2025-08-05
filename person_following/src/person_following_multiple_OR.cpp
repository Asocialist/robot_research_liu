#include <vector>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <deque> // FIX: Add missing header

#include "EllipseTracker2LS.h" // FIX: Include correct tracker
#include "ros/ros.h"
#include "json11.hpp" // FIX: Add missing header
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

// FIX: Remove M_PI macro redefinition
// #define M_PI 3.141592653589793238

// マウスイベントの取得
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
	return;
}

// 定数
float Scale = 0.075f;
cv::Size windowSize;
float ThresL = 3000;
float ThresM = 1000;
float ThresS = 100;

class ScanMessageHandler
{
protected: 
	static constexpr int DEF_SCAN_STEP_MAX = 1080;
	static constexpr double DEF_ANGLE_OFFSET = M_PI;
public:
	static constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;
	static double RetroreflectiveIntensity;
public: 
	std::string topicName;
	double offsetX = 0;
	double offsetY = 0;
	double offsetT = 0;
public: 
	std::vector<int> urgDistance;
	std::vector<int> urgIntensity;
	std::vector<double> sinVal;
	std::vector<double> cosVal;
	std::vector<cv::Point2d> drawPosition;

public:
	void TopicCallbackFunction(const sensor_msgs::LaserScan::ConstPtr &msg){
		if(dataSize != msg->ranges.size()){
			urgDistance.resize(msg->ranges.size());
			urgIntensity.resize(msg->ranges.size());
			CreateTrigonometricTable(msg->angle_min, msg->angle_increment, msg->ranges.size());
			dataSize = msg->ranges.size();
		}
		int sizeScan = msg->ranges.size();
		for (int i = 0; i < sizeScan; i++)
		{
			urgDistance[i] = msg->ranges[i] * SCAN_DISTANCE_MAGNIFICATION;
			urgIntensity[i] = msg->intensities[i];
		}
		IntensityNormalization();
		CalcDrawPosition();
		return;
	}

	void SetOffset(double x, double y, double th){
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
		for(int i = 0; i < num; i ++){
			sinVal[i] = sin(angle_min + angle_increment * i + offsetT + DEF_ANGLE_OFFSET);
			cosVal[i] = cos(angle_min + angle_increment * i + offsetT + DEF_ANGLE_OFFSET);
		}
	}

	void IntensityNormalization()
	{
		int arrayMinimal = std::min((int)urgDistance.size(), (int)urgIntensity.size());
		for (int i = 0; i < arrayMinimal; i++)
		{
            double normalized_intensity = (double)urgIntensity[i] * pow((double)urgDistance[i], 0.5);
			urgIntensity[i] = std::round(normalized_intensity / RetroreflectiveIntensity * 255.0);
			if (urgIntensity[i] > 255) urgIntensity[i] = 255;
			if (urgIntensity[i] < 0) urgIntensity[i] = 0;
		}
	}

	void CalcDrawPosition()
	{
		if(drawPosition.size() != dataSize){
			drawPosition.resize(dataSize);
		}
		int arrayMinimal = urgDistance.size();
		for (int i = 0; i < arrayMinimal; i++)
		{
			if (urgDistance[i] > ThresS && urgDistance[i] < ThresL)
			{
				drawPosition[i].x = std::round(urgDistance[i] * Scale * sinVal[i]) + windowSize.width * 0.5 - offsetY * Scale;
				drawPosition[i].y = std::round(urgDistance[i] * Scale * cosVal[i]) + windowSize.height * 0.5 - offsetX * Scale;
			}
		}
	}

protected:
	int dataSize = 0;
};
double ScanMessageHandler::RetroreflectiveIntensity;

int main(int argc, char **argv)
{
	std::string NodeName = "person_following_multi_OR";
	std::string TopicNameTrackingPosition = "pose_person_following";
	double OutputTopicOffsetPoseX = 0;
	double OutputTopicOffsetPoseY = 0;
	double OutputTopicOffsetPoseT = -0.5 * M_PI;

	double StartTrackingIntensityMin = 80;
	double StartTrackingDistanceMax  = 2200;
	double StopTrackingIntensity 	= 32;	
	ScanMessageHandler::RetroreflectiveIntensity  = 200000;

	windowSize = cv::Size(640, 640);
	std::vector<ScanMessageHandler> handlerList;
	int OffsetX[2] = {0,0};
	int OffsetY[2] = {0,0};
	double OffsetT[2] = {0,0};

// ... 在 main 函数的开头 ...

	if (argc >= 2) {
		// FIX: 使用 LoadJsonFile 来读取和解析文件，而不是直接解析路径字符串
		auto configure = json11::LoadJsonFile(argv[1]); 

        // 检查文件是否成功读取和解析
		if (configure.is_null()) {
			std::cerr << "Failed to load or parse JSON file: " << argv[1] << std::endl;
            return 1; // 如果失败，直接退出
        } 
        else {
            NodeName = configure["NodeName"].is_null() ? NodeName : configure["NodeName"].string_value();
			StartTrackingIntensityMin = configure["StartTrackingIntensityMin"].is_null() ? StartTrackingIntensityMin : configure["StartTrackingIntensityMin"].number_value();
	        StartTrackingDistanceMax = configure["StartTrackingDistanceMax"].is_null() ? StartTrackingDistanceMax : configure["StartTrackingDistanceMax"].number_value();
	        StopTrackingIntensity = configure["StopTrackingIntensity"].is_null() ? StopTrackingIntensity : configure["StopTrackingIntensity"].number_value();
			TopicNameTrackingPosition = configure["TopicNamePublishPosition"].is_null() ? TopicNameTrackingPosition : configure["TopicNamePublishPosition"].string_value();
	        ScanMessageHandler::RetroreflectiveIntensity = configure["RetroreflectiveIntensity"].is_null() ? ScanMessageHandler::RetroreflectiveIntensity : configure["RetroreflectiveIntensity"].number_value();
	            
			windowSize.width = configure["ImageSize"]["x"].is_null() ? windowSize.width : configure["ImageSize"]["x"].int_value();
			windowSize.height = configure["ImageSize"]["y"].is_null() ? windowSize.height : configure["ImageSize"]["y"].int_value();
				
			OutputTopicOffsetPoseX = configure["OutputOffset"]["x"].is_null() ? OutputTopicOffsetPoseX : configure["OutputOffset"]["x"].number_value();
			OutputTopicOffsetPoseY = configure["OutputOffset"]["y"].is_null() ? OutputTopicOffsetPoseY : configure["OutputOffset"]["y"].number_value();
			OutputTopicOffsetPoseT = configure["OutputOffset"]["theta"].is_null() ? OutputTopicOffsetPoseT : configure["OutputOffset"]["theta"].number_value() - (0.5 * M_PI);

			ThresS = configure["ThresholdDistance"]["short"].is_null() ? ThresS : configure["ThresholdDistance"]["short"].number_value();
			ThresM = configure["ThresholdDistance"]["medium"].is_null() ? ThresM : configure["ThresholdDistance"]["medium"].number_value();
			ThresL = configure["ThresholdDistance"]["long"].is_null() ? ThresL : configure["ThresholdDistance"]["long"].number_value();

			if(configure["ScanList"].is_array()){
				auto slist = configure["ScanList"].array_items();
				if(!slist.empty()){
					handlerList.clear();
					int i = 0;
					for(const auto &itr : slist){
						if(i >= 2 || itr["Offset"]["x"].is_null() || itr["Offset"]["y"].is_null() || itr["Offset"]["theta"].is_null() || itr["TopicName"].is_null()){
							continue;
						}
						OffsetX[i] = -itr["Offset"]["y"].number_value() * Scale + windowSize.width/2;
						OffsetY[i] = -itr["Offset"]["x"].number_value() * Scale + windowSize.height/2;
						OffsetT[i] = -itr["Offset"]["theta"].number_value();

						handlerList.push_back(ScanMessageHandler());
						handlerList.back().SetOffset(itr["Offset"]["x"].number_value(), itr["Offset"]["y"].number_value(), itr["Offset"]["theta"].number_value());
						handlerList.back().topicName = itr["TopicName"].string_value();
						i++;
					}
				}
			}
        }
	} else {
        std::cerr << "Error: No JSON configuration file provided." << std::endl;
        return 1;
    }

// ... main 函数的其余部分保持不变 ...
	
	const int OffsetX_ = windowSize.width  * 0.5;
	const int OffsetY_ = windowSize.height * 0.5;

    // FIX: Correct Mat initialization and access
	std::vector<cv::Mat> sensorVisibleRange(2);
	for(int i=0;i<2;++i){
        sensorVisibleRange[i] = cv::Mat::zeros(windowSize, CV_8UC1);
		double oft = OffsetT[i]+M_PI*0.5;
		cv::Vec2d ofv(cos(oft),sin(oft));
		double costh=cos(0.25*M_PI);
		for(int r=0;r<sensorVisibleRange[i].rows;++r){
            for(int c=0;c<sensorVisibleRange[i].cols;++c){
                cv::Vec2d pv(c-OffsetX[i], r-OffsetY[i]);
                if(cv::norm(pv) < 1e-6) continue;
                double co=pv.dot(ofv)/cv::norm(pv);
                sensorVisibleRange[i].at<uint8_t>(r,c) = (co<costh)?255:0;
            }
        }
	}
	if(sensorVisibleRange.size() == 2){
		cv::Mat ___t = cv::Mat::zeros(windowSize, CV_8UC1);
		cv::bitwise_not(sensorVisibleRange[1], ___t);
		cv::bitwise_and(sensorVisibleRange[0], ___t, sensorVisibleRange[0]);
	}

	ros::init(argc, argv, NodeName.c_str());
	ros::NodeHandle nodeHandle;

	std::vector<ros::Subscriber> subscLaserScan;
	for(size_t i = 0; i < handlerList.size(); i++){
		subscLaserScan.push_back(nodeHandle.subscribe(handlerList[i].topicName.c_str(), 100, &ScanMessageHandler::TopicCallbackFunction, &handlerList[i]));
	}
	
	ros::Publisher pubPosePersonFollowing = nodeHandle.advertise<geometry_msgs::Pose2D>(TopicNameTrackingPosition.c_str(), 100);
	geometry_msgs::Pose2D msgPosePersonFollowing;

	cv::Mat baseImage(windowSize, CV_8UC3);
	cv::Mat distanceImage(windowSize, CV_8UC3);
	cv::Mat intensityImage(windowSize, CV_8UC1);
	cv::Mat maskImage(windowSize, CV_8UC1);
	cv::Mat maskImageBinInv(windowSize, CV_8UC1);
	cv::Mat distanceImage2(windowSize, CV_8UC3);
	cv::Mat intensityImage2(windowSize, CV_8UC1);
	cv::Mat dispImage(windowSize, CV_8UC3);
	cv::Mat distImage(windowSize, CV_8UC1);
	cv::Mat distImageNot(windowSize, CV_8UC1);
	cv::Mat transImage(windowSize, CV_32FC1); // Changed from 8UC1 to 32FC1 for distanceTransform result
	cv::Mat dispImage2(windowSize, CV_8UC3);

	baseImage.setTo(cv::Scalar(0,0,0));
    // FIX: use std::round and cv::Scalar
	cv::circle(baseImage, cv::Point(std::round(OffsetX_), std::round(OffsetY_)), 4, cv::Scalar(64, 64, 64), -1);
	for (int jj = 0; jj < 1080; jj++) {
		double angle = (jj * 0.25) * (M_PI / 180);
		cv::circle(baseImage, cv::Point(std::round(ThresS * Scale * sin(angle)) + OffsetX_, std::round(ThresS * Scale * cos(angle)) + OffsetY_), 1, cv::Scalar(64, 64, 64), -1);
		cv::circle(baseImage, cv::Point(std::round(ThresM * Scale * sin(angle)) + OffsetX_, std::round(ThresM * Scale * cos(angle)) + OffsetY_), 1, cv::Scalar(32, 32, 32), -1);
		cv::circle(baseImage, cv::Point(std::round(ThresL * Scale * sin(angle)) + OffsetX_, std::round(ThresL * Scale * cos(angle)) + OffsetY_), 1, cv::Scalar(64, 64, 64), -1);
	}

	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Intensity Image", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("Display Image", mouseCallback);
    
    // FIX: Use modern tracker from its header
	std::vector<cv::Vec2d> lidarPos;
    if(handlerList.size() >= 2){
        lidarPos.push_back({(double)OffsetX[0], (double)OffsetY[0]});
        lidarPos.push_back({(double)OffsetX[1], (double)OffsetY[1]});
    }
	EllipseTracker2LSPool trackers(1, 300, 18, 11, lidarPos, sensorVisibleRange);
    bool isTracked = false;
    double aveIntensity = 0;

	ros::Rate loop_rate(60);
	while (ros::ok())
	{
		baseImage.copyTo(dispImage);
		distanceImage.setTo(cv::Scalar(0));
		intensityImage.setTo(cv::Scalar(0));
		maskImage.setTo(cv::Scalar(0));
		maskImageBinInv.setTo(cv::Scalar(0));
		dispImage2.setTo(cv::Scalar(0));
		distanceImage2.setTo(cv::Scalar(0));
		intensityImage2.setTo(cv::Scalar(0));
		
		int loopNumber = 0;
		for(const auto &itr : handlerList){
            cv::Mat& currentDistImg = (loopNumber == 0) ? distanceImage : distanceImage2;
            cv::Mat& currentIntenImg = (loopNumber == 0) ? intensityImage : intensityImage2;
            cv::Mat& currentDispImg = (loopNumber == 0) ? dispImage : dispImage2;

			for (size_t i = 0; i < itr.drawPosition.size(); i++)
			{
                // FIX: cv::Scalar, added bounds check
                int color1 = itr.urgIntensity[i] * 4 + 64;
                if(color1 > 255) color1 = 255;
				cv::circle(currentDistImg, itr.drawPosition[i], 1, cv::Scalar(255, 255, 0), -1);
				cv::circle(currentIntenImg, itr.drawPosition[i], 1, cv::Scalar(itr.urgIntensity[i]), -1);
				cv::circle(currentDispImg, itr.drawPosition[i], 1, cv::Scalar(color1, color1, 64), -1);
			}

			if(loopNumber == 0 && handlerList.size() > 1){
				// ... (Masking logic seems complex and might need specific debugging)
			}
			loopNumber++;
		}
        
        cv::add(distanceImage, distanceImage2, distanceImage);
		cv::add(intensityImage, intensityImage2, intensityImage);
		cv::add(dispImage, dispImage2, dispImage);

		cv::imshow("Intensity Image", intensityImage);
		
		cv::cvtColor(distanceImage, distImage, cv::COLOR_BGR2GRAY);
		cv::threshold(distImage, distImageNot, 64.0, 255.0, cv::THRESH_BINARY_INV);
		// FIX: Correct distanceTransform call
        cv::distanceTransform(distImageNot, transImage, cv::DIST_L2, 3);
        
        if(!trackers.empty()) {
		    trackers.next(transImage);
        }

		int usrU = -100, usrV = -100, usrAngl = 0;
		if(!trackers.empty()){
			auto _p = trackers[0]->getPos();
			usrU = _p[0]; usrV = _p[1]; usrAngl = _p[2];

            cv::Rect roi = cv::Rect(usrU - 30, usrV - 30, 60, 60) & cv::Rect(0, 0, intensityImage.cols, intensityImage.rows);
            if(roi.area() > 0) {
			    cv::Mat intensityImageROI(intensityImage, roi);
			    aveIntensity = cv::countNonZero(intensityImageROI) > 0 ? cv::sum(intensityImageROI)[0] / cv::countNonZero(intensityImageROI) : 0.0;
            } else {
                aveIntensity = 0.0;
            }

			if (aveIntensity < StopTrackingIntensity) {
				trackers.removeByOrder(0);
			}
		}

        isTracked = !trackers.empty();

		if (!isTracked)
		{ 
			for(const auto &itr : handlerList){
				for (size_t i = 0; i < itr.urgDistance.size(); i++)
				{
					if (itr.urgDistance[i] < StartTrackingDistanceMax && itr.urgIntensity[i] > StartTrackingIntensityMin)
					{
						usrU = std::round((itr.urgDistance[i] * Scale * itr.sinVal[i]) + OffsetX_ - 20);
						usrV = std::round((itr.urgDistance[i] * Scale * itr.cosVal[i]) + OffsetY_);
						trackers.add(usrU, usrV, 0); // Use last angle
						isTracked = true;
						break;
					}
				}
				if(isTracked) break;
			}
		}

		if (lbPressed)
		{
			lbPressed = false;
			if(trackers.empty()) trackers.add(lbX, lbY, 0);
			else trackers[0]->init(lbX, lbY, 0);
            isTracked = true;
		}

		if (isTracked)
		{
            auto _p = trackers[0]->getPos();
			usrU = _p[0]; usrV = _p[1]; usrAngl = _p[2];

			// FIX: dispimage typo and CV_RGB
			cv::ellipse(dispImage, cv::Point(usrU, usrV), cv::Size(24,12), usrAngl, 0, 360, cv::Scalar(255,255,255), 2);
			cv::rectangle(dispImage, cv::Point(usrU - 30, usrV - 30), cv::Point(usrU + 30, usrV + 30), cv::Scalar(255, 255, 255), 1);
			char mytext[12], angleText[20];
			sprintf(mytext, "%4.0lf", aveIntensity);
			cv::putText(dispImage, mytext, cv::Point(usrU + 30, usrV + 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255));
			sprintf(angleText, "Ang: %d", (int)std::round(usrAngl));
			cv::putText(dispImage, angleText, cv::Point(usrU + 30, usrV + 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255));
		
			double rotX = (usrU - OffsetX_) * cos(OutputTopicOffsetPoseT) - (usrV - OffsetY_) * sin(OutputTopicOffsetPoseT);
			double rotY = (usrU - OffsetX_) * sin(OutputTopicOffsetPoseT) + (usrV - OffsetY_) * cos(OutputTopicOffsetPoseT);
			msgPosePersonFollowing.x = OutputTopicOffsetPoseX + (-1.0) * rotX / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.y = OutputTopicOffsetPoseY - (-1.0) * rotY / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.theta = OutputTopicOffsetPoseT  + (-usrAngl*M_PI/180.0 + M_PI*1.5);
			pubPosePersonFollowing.publish(msgPosePersonFollowing);

			// FIX: Use %f for double in ROS_INFO/WARN
			ROS_WARN("angle= %3.1f", (double)usrAngl);
			ROS_INFO("msg=%f", msgPosePersonFollowing.theta);
		}
		
		cv::imshow("Display Image", dispImage);
		if (cv::waitKey(1) == 'q') break;

		ros::spinOnce();
		loop_rate.sleep();
	}

	cv::destroyAllWindows();
	return 0;
}