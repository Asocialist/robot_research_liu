#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "MyEllipseNormalEvaluation_2LS.h" // 2LS(2 Laser Scanner)版の評価関数ヘッダ
#include "MyCondensation.h"
#include "json11.hpp"

// --- 定数 ---
constexpr double M_PI_VAL = 3.14159265358979;

// --- グローバル変数 ---
bool g_lbPressed = false;
cv::Point g_mouseClickPos;
float g_scale = 0.075f;
cv::Size g_windowSize; // [修正] cv::Point から cv::Size へ変更
float g_thresL = 3000.0f;
float g_thresS = 100.0f;

// --- マウスコールバック ---
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        g_lbPressed = true;
        g_mouseClickPos = cv::Point(x, y);
    }
}

// --- ScanMessageHandler クラス ---
class ScanMessageHandler {
public:
    static double s_retroreflectiveIntensity;
    static constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;

    std::string topicName;
    double offsetX = 0.0, offsetY = 0.0, offsetT = 0.0;

    std::vector<int> urgDistance;
    std::vector<int> urgIntensity;
    std::vector<cv::Point2d> drawPosition;

    ScanMessageHandler() : dataSize(0) {}

    void setOffset(double x, double y, double th) {
        offsetX = x;
        offsetY = y;
        offsetT = th;
        dataSize = 0;
    }

    void topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (dataSize != msg->ranges.size()) {
            urgDistance.resize(msg->ranges.size());
            urgIntensity.resize(msg->ranges.size());
            createTrigonometricTable(msg->angle_min, msg->angle_increment, msg->ranges.size());
            dataSize = msg->ranges.size();
        }

        for (int i = 0; i < dataSize; ++i) {
            urgDistance[i] = msg->ranges[i] * SCAN_DISTANCE_MAGNIFICATION;
            urgIntensity[i] = msg->intensities[i];
        }

        intensityNormalization();
        calcDrawPosition();
    }

private:
    int dataSize;
    std::vector<double> sinVal;
    std::vector<double> cosVal;

    void createTrigonometricTable(double angle_min, double angle_increment, int num) {
        sinVal.resize(num);
        cosVal.resize(num);
        for (int i = 0; i < num; ++i) {
            double angle = angle_min + angle_increment * i + offsetT + M_PI_VAL;
            sinVal[i] = sin(angle);
            cosVal[i] = cos(angle);
        }
    }

    void intensityNormalization() {
        if (dataSize < 2) return;
        std::vector<double> URG_Intensity_I(dataSize);
        for (int i = 0; i < dataSize; i++) {
            if (urgDistance[i] <= 0) {
                urgIntensity[i] = 0;
                continue;
            }
            int bi = (i > 0) ? i - 1 : i + 1;
            int ni = (i < dataSize - 1) ? i + 1 : i - 1;

            cv::Vec2d dp(urgDistance[i] * sinVal[i], urgDistance[i] * cosVal[i]);
            cv::Vec2d bdp(urgDistance[bi] * sinVal[bi], urgDistance[bi] * cosVal[bi]);
            cv::Vec2d ndp(urgDistance[ni] * sinVal[ni], urgDistance[ni] * cosVal[ni]);
            
            cv::Vec2d viewv = dp; 
            if (cv::norm(viewv) < 1e-6) continue;
            viewv /= cv::norm(viewv);

            cv::Vec2d np;
            const double DP_THRESH = 200;
            cv::Vec2d bv = bdp - dp;
            double bnl = cv::norm(bv);
            cv::Vec2d nv = ndp - dp;
            double nnl = cv::norm(nv);

            if (bnl > DP_THRESH && nnl > DP_THRESH) np = viewv;
            else if (bnl > DP_THRESH) np = cv::Vec2d(-nv[1], nv[0]);
            else if (nnl > DP_THRESH) np = cv::Vec2d(-bv[1], bv[0]);
            else np = cv::Vec2d(-nv[1], nv[0]) + cv::Vec2d(-bv[1], bv[0]);
            
            if (cv::norm(np) < 1e-6) continue;
            np /= cv::norm(np);
            
            double angle_cos = np.dot(viewv);
            URG_Intensity_I[i] = urgIntensity[i] * pow((double)urgDistance[i], 0.5);
            int final_intensity = cvRound(URG_Intensity_I[i] / s_retroreflectiveIntensity * 255.0);
            urgIntensity[i] = std::max(0, std::min(255, final_intensity));
        }
    }

    void calcDrawPosition() {
        if (drawPosition.size() != dataSize) drawPosition.resize(dataSize);
        for (int i = 0; i < dataSize; ++i) {
            drawPosition[i].x = g_windowSize.width / 2.0 + (urgDistance[i] * sinVal[i] - offsetY) * g_scale;
            drawPosition[i].y = g_windowSize.height / 2.0 - (urgDistance[i] * cosVal[i] + offsetX) * g_scale;
        }
    }
};
double ScanMessageHandler::s_retroreflectiveIntensity = 200000.0;

int main(int argc, char** argv) {
    // --- 設定 ---
    std::string nodeName = "person_following_multi_OR";
    std::string topicNameTrackingPosition = "pose_person_following";
    double outputTopicOffsetPoseX = 0.0, outputTopicOffsetPoseY = 0.0, outputTopicOffsetPoseT = -0.5 * M_PI_VAL;
    double startTrackingIntensityMin = 80.0, startTrackingDistanceMax = 2200.0, stopTrackingIntensity = 32.0;
    
    g_windowSize = cv::Size(640, 640); // [修正] cv::Point から cv::Size へ変更
    std::vector<ScanMessageHandler> handlerList(2);

    // --- JSONファイルからの設定読み込み ---
    if (argc >= 2) {
        std::ifstream ifs(argv[1]);
        if (!ifs.is_open()) {
            std::cerr << "Cannot open config file: " << argv[1] << std::endl;
            return 1;
        }
        std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        std::string err;
        auto json = json11::Json::parse(content, err);
        if (!err.empty()) {
            std::cerr << "Cannot parse config file: " << argv[1] << " Error: " << err << std::endl;
            return 1;
        }
        
        nodeName = json["NodeName"].string_value();
        startTrackingIntensityMin = json["StartTrackingIntensityMin"].number_value();
        startTrackingDistanceMax = json["StartTrackingDistanceMax"].number_value();
        stopTrackingIntensity = json["StopTrackingIntensity"].number_value();
        ScanMessageHandler::s_retroreflectiveIntensity = json["RetroreflectiveIntensity"].number_value();
        g_windowSize.width = json["ImageSize"]["x"].int_value(); // [修正] .x から .width
        g_windowSize.height = json["ImageSize"]["y"].int_value(); // [修正] .y から .height
        g_thresS = json["ThresholdDistance"]["short"].number_value();
        g_thresL = json["ThresholdDistance"]["long"].number_value();
        
        const auto& slist = json["ScanList"].array_items();
        if (slist.size() >= 2) {
            for(int i = 0; i < 2; ++i) {
                handlerList[i].setOffset(slist[i]["Offset"]["x"].number_value(), slist[i]["Offset"]["y"].number_value(), slist[i]["Offset"]["theta"].number_value());
                handlerList[i].topicName = slist[i]["TopicName"].string_value();
            }
        }
    } else {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        return 1;
    }

    // --- ROS初期化 ---
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscLaserScan;
    for (size_t i = 0; i < handlerList.size(); ++i) {
        subscLaserScan.push_back(nh.subscribe(handlerList[i].topicName, 100, &ScanMessageHandler::topicCallback, &handlerList[i]));
    }
    ros::Publisher pubPosePersonFollowing = nh.advertise<geometry_msgs::Pose2D>(topicNameTrackingPosition, 100);

    // --- OpenCVウィンドウと2LS評価関数の準備 ---
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Display Image", mouseCallback);

    cv::Mat visibleMask[2];
    for(int i=0; i<2; ++i) {
        cv::Point sensorPosOnImage(
            g_windowSize.width / 2.0 - handlerList[i].offsetY * g_scale,
            g_windowSize.height / 2.0 - handlerList[i].offsetX * g_scale
        );
        SetSensorPosition_2LS(i, sensorPosOnImage);

        visibleMask[i] = cv::Mat::zeros(g_windowSize, CV_8U);
        double oft = -handlerList[i].offsetT + M_PI_VAL * 0.5;
        cv::Vec2d ofv(cos(oft), sin(oft));
        double costh = cos(135.0 * M_PI_VAL / 180.0); // 270度FOV
        for (int r = 0; r < g_windowSize.height; ++r) { // [修正] g_windowSize.y -> g_windowSize.height
            for (int c = 0; c < g_windowSize.width; ++c) { // [修正] g_windowSize.x -> g_windowSize.width
                cv::Vec2d pv(c - sensorPosOnImage.x, r - sensorPosOnImage.y);
                if (cv::norm(pv) > 1e-6) {
                     if (pv.dot(ofv) / cv::norm(pv) > costh) {
                        visibleMask[i].at<uchar>(r, c) = 255;
                    }
                }
            }
        }
    }
    cv::Mat mask0_only, mask1_only;
    cv::bitwise_and(visibleMask[0], ~visibleMask[1], mask0_only);
    cv::bitwise_and(visibleMask[1], ~visibleMask[0], mask1_only);
    SetSensorVisibleRange_2LS(0, mask0_only);
    SetSensorVisibleRange_2LS(1, mask1_only);

    StoreBodyContourPosition_2LS(18, 11, 10);
    StoreHeadContourPosition_2LS(9, 10);

    // --- パーティクルフィルタ準備 ---
    MyCondensation condensation(3, 300);
    cv::Mat initValue = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);
    cv::Mat initMean = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);
    cv::Mat initDeviation = (cv::Mat_<float>(3, 1) << 5.0f, 5.0f, 20.0f);
    condensation.initSampleSet(initValue, initMean, initDeviation);
    
    bool isTracked = false;
    int prevusrAngl = 0;

    // --- メインループ ---
    ros::Rate loop_rate(40);
    while (ros::ok()) {
        cv::Mat baseImage = cv::Mat::zeros(g_windowSize, CV_8UC3);
        cv::Mat dispImage, distanceImage, intensityImage;
        baseImage.copyTo(dispImage);
        
        cv::Mat distanceImage1 = cv::Mat::zeros(g_windowSize, CV_8UC3);
        cv::Mat intensityImage1 = cv::Mat::zeros(g_windowSize, CV_8UC3);
        cv::Mat distanceImage2 = cv::Mat::zeros(g_windowSize, CV_8UC3);
        cv::Mat intensityImage2 = cv::Mat::zeros(g_windowSize, CV_8UC3);
        
        for (size_t i = 0; i < handlerList[0].drawPosition.size(); ++i) {
            if (handlerList[0].urgDistance[i] > g_thresS && handlerList[0].urgDistance[i] < g_thresL) {
                cv::circle(distanceImage1, handlerList[0].drawPosition[i], 1, cv::Scalar(255, 255, 0), -1);
                int c = handlerList[0].urgIntensity[i];
                cv::circle(intensityImage1, handlerList[0].drawPosition[i], 1, cv::Scalar(c, c, c), -1);
            }
        }
        for (size_t i = 0; i < handlerList[1].drawPosition.size(); ++i) {
            if (handlerList[1].urgDistance[i] > g_thresS && handlerList[1].urgDistance[i] < g_thresL) {
                cv::circle(distanceImage2, handlerList[1].drawPosition[i], 1, cv::Scalar(255, 255, 0), -1);
                int c = handlerList[1].urgIntensity[i];
                cv::circle(intensityImage2, handlerList[1].drawPosition[i], 1, cv::Scalar(c, c, c), -1);
            }
        }

        distanceImage1.copyTo(distanceImage);
        distanceImage2.copyTo(distanceImage, visibleMask[1]);
        intensityImage1.copyTo(intensityImage);
        intensityImage2.copyTo(intensityImage, visibleMask[1]);

        cv::Mat distImageGray, distImageInv, distTransformedImg;
        cv::cvtColor(distanceImage, distImageGray, cv::COLOR_BGR2GRAY);
        cv::threshold(distImageGray, distImageInv, 1, 255.0, cv::THRESH_BINARY_INV);
        cv::distanceTransform(distImageInv, distTransformedImg, cv::DIST_L2, 3);

        condensation.updateSample();
        for (size_t i = 0; i < condensation.samples.size(); ++i) {
            cv::Point center(cvRound(condensation.samples[i].at<float>(0)), cvRound(condensation.samples[i].at<float>(1)));
            int angle = cvRound(condensation.samples[i].at<float>(2));
            condensation.confidence[i] = CalculateBodyLikelihood_2LS(distTransformedImg, center, angle);
        }
        condensation.updateByTime();

        cv::Point usrPos(cvRound(condensation.state.at<float>(0)), cvRound(condensation.state.at<float>(1)));
        int usrAngl = static_cast<int>(condensation.state.at<float>(2)) % 360;
        if(usrAngl < 0) usrAngl += 360;

        int aveIntensity = 0;
        cv::Rect roi(usrPos.x - 30, usrPos.y - 30, 60, 60);
        roi &= cv::Rect(0, 0, g_windowSize.width, g_windowSize.height); // [修正] .x, .y -> .width, .height
        if (roi.area() > 0) {
            cv::Mat intensityGray;
            cv::cvtColor(intensityImage, intensityGray, cv::COLOR_BGR2GRAY);
            int nonZero = cv::countNonZero(intensityGray(roi));
            aveIntensity = (nonZero > 0) ? (cv::sum(intensityGray(roi))[0] / nonZero) : 0;
        }

        if (isTracked && aveIntensity < stopTrackingIntensity) isTracked = false;

        if (!isTracked) {
             for (const auto& handler : handlerList) {
                for (size_t i = 0; i < handler.urgDistance.size(); ++i) {
                    if (handler.urgDistance[i] < startTrackingDistanceMax && handler.urgIntensity[i] > startTrackingIntensityMin) {
                        initValue.at<float>(0) = handler.drawPosition[i].x;
                        initValue.at<float>(1) = handler.drawPosition[i].y;
                        initValue.at<float>(2) = prevusrAngl;
                        condensation.initSampleSet(initValue, initMean, initDeviation);
                        isTracked = true;
                        goto found_target_or_final;
                    }
                }
            }
        }
        found_target_or_final:;

        if (g_lbPressed) {
            g_lbPressed = false;
            initValue.at<float>(0) = (float)g_mouseClickPos.x;
            initValue.at<float>(1) = (float)g_mouseClickPos.y;
            condensation.initSampleSet(initValue, initMean, initDeviation);
            usrPos = g_mouseClickPos;
            isTracked = true;
        }

        if (isTracked) {
            DrawBodyContour_2LS(dispImage, usrPos, usrAngl);
            cv::rectangle(dispImage, roi, cv::Scalar(255, 255, 255), 1);
            cv::putText(dispImage, "Ang: " + std::to_string(usrAngl), usrPos + cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255));
            
            double img_x = usrPos.x - g_windowSize.width / 2.0; // [修正]
            double img_y = usrPos.y - g_windowSize.height / 2.0; // [修正]
            double robot_x_mm = -img_y / g_scale;
            double robot_y_mm = img_x / g_scale;

            geometry_msgs::Pose2D msg;
            msg.x = robot_x_mm / 1000.0;
            msg.y = robot_y_mm / 1000.0;
            msg.theta = -usrAngl * M_PI_VAL / 180.0;
            pubPosePersonFollowing.publish(msg);
            
            prevusrAngl = usrAngl;
        }

        cv::imshow("Display Image", dispImage);
        if (cv::waitKey(1) == 'q') break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
}
