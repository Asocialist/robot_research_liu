//2台のLiDARのmsgから人を追跡する

#include <vector>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "EllipseTracker.h"

#include "ros/ros.h"
#include "json11.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"


//  #define M_PI 3.141592653589793238

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
float Scale = 0.1f;
cv::Size windowSize;

class ScanMessageHnadler
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
			urgIntensity[i] = std::round(normalized_intensity / RetroreflectiveIntensity * 255);
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
            // FIX: std::round
			drawPosition[i].x = -std::round(urgDistance[i] * Scale * sinVal[i]) + windowSize.width  * 0.5 - offsetY * Scale;
			drawPosition[i].y =  std::round(urgDistance[i] * Scale * cosVal[i]) + windowSize.height * 0.5 - offsetX * Scale;
		}
	}

protected:
	int dataSize = 0;
};
double ScanMessageHnadler::RetroreflectiveIntensity;

int main(int argc, char **argv)
{
	// ---Configure---
	std::string NodeName = "person_following_multi";
	std::string TopicNameTrackingPosition = "pose_person_following";
	double OutputTopicOffsetPoseX = 0;
	double OutputTopicOffsetPoseY = 0;
	double OutputTopicOffsetPoseT = -0.5 * M_PI;
	double StartTrackingIntensityMin = 80;
	double StartTrackingDistanceMax  = 2200;
	double StopTrackingIntensity 	= 32;	
	ScanMessageHnadler::RetroreflectiveIntensity  = 200000;

	windowSize = cv::Size(640, 640);
	float ThresL = 3000;
	float ThresM = 1000;
	float ThresS = 200;

	std::vector<ScanMessageHnadler> handlerList;
	handlerList.push_back(ScanMessageHnadler());
	handlerList.back().SetOffset(0, 0, 0);
	handlerList.back().topicName = "scan";

	{// ---Load configure---
	    if (argc >= 2) {
	        std::string err;
	        auto configure = json11::Json::parse(std::string(argv[1]), err);
	        if (!err.empty()) {
	            std::cerr << "Cannot read or parse configure file: " << err << std::endl;
	        }
	        else{
	            NodeName = configure["NodeName"].is_null() ? NodeName : configure["NodeName"].string_value();
				StartTrackingIntensityMin = configure["StartTrackingIntensityMin"].is_null() ? StartTrackingIntensityMin : configure["StartTrackingIntensityMin"].number_value();
	            StartTrackingDistanceMax = configure["StartTrackingDistanceMax"].is_null() ? StartTrackingDistanceMax : configure["StartTrackingDistanceMax"].number_value();
	            StopTrackingIntensity = configure["StopTrackingIntensity"].is_null() ? StopTrackingIntensity : configure["StopTrackingIntensity"].number_value();
				TopicNameTrackingPosition = configure["TopicNamePublishPosition"].is_null() ? TopicNameTrackingPosition : configure["TopicNamePublishPosition"].string_value();
	            ScanMessageHnadler::RetroreflectiveIntensity = configure["RetroreflectiveIntensity"].is_null() ? ScanMessageHnadler::RetroreflectiveIntensity : configure["RetroreflectiveIntensity"].number_value();
	            
				windowSize.width  = configure["ImageSize"]["x"].is_null() ? windowSize.width  : configure["ImageSize"]["x"].int_value();
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
						for(const auto &itr : slist){
							if(itr["Offset"]["x"].is_null() || itr["Offset"]["y"].is_null() || itr["Offset"]["theta"].is_null() || itr["TopicName"].is_null()){
								continue;
							}
							handlerList.push_back(ScanMessageHnadler());
							handlerList.back().SetOffset(itr["Offset"]["x"].number_value(), itr["Offset"]["y"].number_value(), itr["Offset"]["theta"].number_value());
							handlerList.back().topicName = itr["TopicName"].string_value();
						}
					}
				}
	        }
	    }
	}

	const int OffsetX = windowSize.width * 0.5;
	const int OffsetY = windowSize.height * 0.5;

	ros::init(argc, argv, NodeName.c_str());
	ros::NodeHandle nodeHandle;

	std::vector<ros::Subscriber> subscLaserScan;
	for(size_t i = 0; i < handlerList.size(); i++){
		subscLaserScan.push_back(nodeHandle.subscribe(handlerList[i].topicName.c_str(), 100, &ScanMessageHnadler::TopicCallbackFunction, &handlerList[i]));
	}
	
	ros::Publisher pubPosePersonFollowing = nodeHandle.advertise<geometry_msgs::Pose2D>(TopicNameTrackingPosition.c_str(), 100);
	geometry_msgs::Pose2D msgPosePersonFollowing;

	cv::Mat baseImage      (windowSize, CV_8UC3);
	cv::Mat distanceImage  (windowSize, CV_8UC3);
	cv::Mat intensityImage (windowSize, CV_8UC1);
	cv::Mat dispImage    (windowSize, CV_8UC3);	  
	cv::Mat distImage    (windowSize, CV_8UC1);	  
	cv::Mat distImageNot (windowSize, CV_8UC1); 
	cv::Mat transImage   (windowSize, CV_32FC1); 

	baseImage.setTo(cv::Scalar(0,0,0));
	cv::circle(baseImage, cv::Point(OffsetX, OffsetY), 4, cv::Scalar(64, 64, 64), -1); 
	for (int jj = 0; jj < 1080; jj++)
	{
		double angle = (jj * 0.25) * (M_PI / 180);
        // FIX: std::round and cv::Scalar
		cv::circle(baseImage, cv::Point(std::round(ThresS * Scale * sin(angle)) + OffsetX, std::round(ThresS * Scale * cos(angle)) + OffsetY), 1, cv::Scalar(64, 64, 64), -1);
		cv::circle(baseImage, cv::Point(std::round(ThresM * Scale * sin(angle)) + OffsetX, std::round(ThresM * Scale * cos(angle)) + OffsetY), 1, cv::Scalar(32, 32, 32), -1);
		cv::circle(baseImage, cv::Point(std::round(ThresL * Scale * sin(angle)) + OffsetX, std::round(ThresL * Scale * cos(angle)) + OffsetY), 1, cv::Scalar(64, 64, 64), -1);
	}

	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Intensity Image", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("Display Image", mouseCallback);

	EllipseTrackerPool trackers(1, 300, 24, 12, cv::Vec2d(OffsetX, OffsetY));
    bool isTracked = false;
    double aveIntensity = 0;
	
	ros::Rate loop_rate(60);
	while (ros::ok())
	{
		baseImage.copyTo(dispImage);
		distanceImage.setTo(cv::Scalar(0));
		intensityImage.setTo(cv::Scalar(0));
		
		for(const auto &itr : handlerList){
			for (size_t i = 0; i < itr.drawPosition.size(); i++)
			{
				if (itr.urgDistance[i] > ThresS && itr.urgDistance[i] < ThresL)
				{
                    // FIX: cv::Scalar
					cv::circle(distanceImage, itr.drawPosition[i], 1, cv::Scalar(255, 255, 0), -1);
					cv::circle(intensityImage, itr.drawPosition[i], 1, cv::Scalar(itr.urgIntensity[i]), -1);
					int color1 = itr.urgIntensity[i] * 4 + 64;
                    if(color1 > 255) color1 = 255;
					cv::circle(dispImage, itr.drawPosition[i], 1, cv::Scalar(color1, color1, 64), -1);
				}
			}
		}

		cv::imshow("Intensity Image", intensityImage);
        
		cv::cvtColor(distanceImage, distImage, cv::COLOR_BGR2GRAY);
		cv::threshold(distImage, distImageNot, 64.0, 255.0, cv::THRESH_BINARY_INV);
        // FIX: Use correct distanceTransform
		cv::distanceTransform(distImageNot, transImage, cv::DIST_L2, 3);

		if(!trackers.empty()) {
            trackers.next(transImage);
        }

		int usrU = -100, usrV = -100, usrAngl = 0;
		if(!trackers.empty()){
            // FIX: Correctly get position from tracker object, not pool
			auto _p = trackers[0]->getPos(); 
			usrU = _p[0]; usrV = _p[1]; usrAngl = _p[2];

            // FIX: ROI must be within image bounds
            cv::Rect roi = cv::Rect(usrU - 30, usrV - 30, 60, 60) & cv::Rect(0, 0, intensityImage.cols, intensityImage.rows);
            if(roi.area() > 0){
			    cv::Mat intensityROI(intensityImage, roi);
			    aveIntensity = cv::countNonZero(intensityROI) > 0 ? cv::sum(intensityROI)[0] / cv::countNonZero(intensityROI) : 0;
            } else {
                aveIntensity = 0;
            }

            if(aveIntensity < StopTrackingIntensity) {
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
                        // FIX: std::round
						int initU = std::round((itr.urgDistance[i] * Scale * itr.sinVal[i]) + OffsetX - 20);
						int initV = std::round((itr.urgDistance[i] * Scale * itr.cosVal[i]) + OffsetY);
						trackers.add(initU, initV, 0);
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
            // Re-fetch position after potential update
            auto _p = trackers[0]->getPos(); 
			usrU = _p[0]; usrV = _p[1]; usrAngl = _p[2];

            // FIX: cv::Scalar
			cv::ellipse(dispImage, cv::Point(usrU, usrV), cv::Size(24,12), usrAngl, 0, 360, cv::Scalar(255,255,255), 2);
			cv::rectangle(dispImage, cv::Point(usrU - 30, usrV - 30), cv::Point(usrU + 30, usrV + 30), cv::Scalar(255, 255, 255), 1);
			
            if (aveIntensity < 0) aveIntensity = 0;
			char mytext[12];
			sprintf(mytext, "%4.0lf", aveIntensity);
			cv::putText(dispImage, mytext, cv::Point(usrU + 30, usrV + 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255), 1);

			double rotX = (usrU - OffsetX) * cos(OutputTopicOffsetPoseT) - (usrV - OffsetY) * sin(OutputTopicOffsetPoseT);
			double rotY = (usrU - OffsetX) * sin(OutputTopicOffsetPoseT) + (usrV - OffsetY) * cos(OutputTopicOffsetPoseT);
			msgPosePersonFollowing.x = OutputTopicOffsetPoseX + (-1.0) * rotX / (ScanMessageHnadler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.y = OutputTopicOffsetPoseY - (-1.0) * rotY / (ScanMessageHnadler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.theta = OutputTopicOffsetPoseT + (usrAngl * M_PI / 180.0); // FIX: Angle conversion
			pubPosePersonFollowing.publish(msgPosePersonFollowing);
		}

		cv::imshow("Display Image", dispImage);
		if (cv::waitKey(1) == 'q') break;

		ros::spinOnce();
		loop_rate.sleep();
	}

	cv::destroyAllWindows();
	return 0;
}