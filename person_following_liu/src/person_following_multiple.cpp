#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "MyEllipseNormalEvaluation.h" // 1センサ版の評価関数
#include "MyCondensation.h"
#include "json11.hpp"

// --- 定数 ---
constexpr double M_PI_VAL = 3.14159265358979;

// --- グローバル変数 ---
bool g_lbPressed = false;
cv::Point g_mouseClickPos;
float g_scale = 0.1f;
cv::Size g_windowSize; // [修正] cv::Point から cv::Size へ変更

// --- マウスコールバック ---
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        g_lbPressed = true;
        g_mouseClickPos = cv::Point(x, y);
    }
}

/**
 * @class ScanMessageHandler
 * @brief 単一のレーザスキャナデータを処理するクラス
 */
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
        // 元のコードの複雑な正規化処理を簡略化
        for (size_t i = 0; i < urgIntensity.size(); ++i) {
            if (urgDistance[i] <= 0) {
                urgIntensity[i] = 0;
                continue;
            }
            double normalized_intensity = (double)urgIntensity[i] * pow((double)urgDistance[i], 0.5);
            int final_intensity = cvRound(normalized_intensity / s_retroreflectiveIntensity * 255.0);
            urgIntensity[i] = std::max(0, std::min(255, final_intensity));
        }
    }

    void calcDrawPosition() {
        if (drawPosition.size() != dataSize) {
            drawPosition.resize(dataSize);
        }
        for (int i = 0; i < dataSize; ++i) {
            // 座標系を修正: ROS(前X,左Y) -> 画像(右X,下Y)
            double robotX = urgDistance[i] * cosVal[i];
            double robotY = urgDistance[i] * sinVal[i];
            drawPosition[i].x = g_windowSize.width / 2.0 - (robotY + offsetY) * g_scale; // [修正]
            drawPosition[i].y = g_windowSize.height / 2.0 - (robotX + offsetX) * g_scale; // [修正]
        }
    }
};
double ScanMessageHandler::s_retroreflectiveIntensity = 200000.0;

int main(int argc, char** argv) {
    // --- 設定 ---
    std::string nodeName = "person_following_multi";
    std::string topicNameTrackingPosition = "pose_person_following";
    double outputTopicOffsetPoseX = 0.0, outputTopicOffsetPoseY = 0.0, outputTopicOffsetPoseT = -0.5 * M_PI_VAL;
    double startTrackingIntensityMin = 80.0, startTrackingDistanceMax = 2200.0, stopTrackingIntensity = 32.0;
    
    g_windowSize = cv::Size(640, 640); // [修正]
    float thresL = 3000.0f, thresM = 1000.0f, thresS = 200.0f;
    std::vector<ScanMessageHandler> handlerList;

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
        g_windowSize.width = json["ImageSize"]["x"].int_value(); // [修正]
        g_windowSize.height = json["ImageSize"]["y"].int_value(); // [修正]
        thresS = json["ThresholdDistance"]["short"].number_value();
        thresM = json["ThresholdDistance"]["medium"].number_value();
        thresL = json["ThresholdDistance"]["long"].number_value();

        const auto& slist = json["ScanList"].array_items();
        for(const auto& item : slist) {
            handlerList.emplace_back();
            handlerList.back().setOffset(item["Offset"]["x"].number_value(), item["Offset"]["y"].number_value(), item["Offset"]["theta"].number_value());
            handlerList.back().topicName = item["TopicName"].string_value();
        }
    } else {
        std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
        // デフォルトのハンドラを1つ追加
        handlerList.emplace_back();
        handlerList.back().setOffset(0, 0, 0);
        handlerList.back().topicName = "scan";
    }

    // --- ROS初期化 ---
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscLaserScan;
    for (size_t i = 0; i < handlerList.size(); ++i) {
        subscLaserScan.push_back(nh.subscribe(handlerList[i].topicName, 100, &ScanMessageHandler::topicCallback, &handlerList[i]));
    }
    ros::Publisher pubPosePersonFollowing = nh.advertise<geometry_msgs::Pose2D>(topicNameTrackingPosition, 100);

    // --- OpenCVウィンドウと描画の準備 ---
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Display Image", mouseCallback);
    cv::Mat baseImage = cv::Mat::zeros(g_windowSize, CV_8UC3);
    // (ベース画像の描画ロジックは簡略化)

    // --- パーティクルフィルタ準備 ---
    MyCondensation condensation(3, 300);
    cv::Mat initValue = (cv::Mat_<float>(3, 1) << g_windowSize.width / 2.0f, g_windowSize.height / 2.0f, 0.0f); // [修正]
    cv::Mat initMean = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);
    cv::Mat initDeviation = (cv::Mat_<float>(3, 1) << 5.0f, 5.0f, 20.0f);
    condensation.initSampleSet(initValue, initMean, initDeviation);
    
    SetSensorPosition(cv::Point(g_windowSize.width / 2, g_windowSize.height / 2)); // [修正]
    StoreBodyContourPosition(24, 12, 10);
    StoreHeadContourPosition(12, 10);

    bool isTracked = false;

    // --- メインループ ---
    ros::Rate loop_rate(40);
    while (ros::ok()) {
        cv::Mat dispImage, distanceImage, intensityImage;
        baseImage.copyTo(dispImage);
        distanceImage = cv::Mat::zeros(g_windowSize, CV_8UC3);
        intensityImage = cv::Mat::zeros(g_windowSize, CV_8UC3);

        for (const auto& handler : handlerList) {
            for (size_t i = 0; i < handler.drawPosition.size(); ++i) {
                if (handler.urgDistance[i] > thresS && handler.urgDistance[i] < thresL) {
                    const auto& pt = handler.drawPosition[i];
                    if (pt.x >= 0 && pt.x < g_windowSize.width && pt.y >= 0 && pt.y < g_windowSize.height) { // [修正]
                        cv::circle(distanceImage, pt, 1, cv::Scalar(255, 255, 0), -1);
                        int c = handler.urgIntensity[i];
                        cv::circle(intensityImage, pt, 1, cv::Scalar(c, c, c), -1);
                        int c1 = std::min(255, c * 4 + 64);
                        cv::circle(dispImage, pt, 1, cv::Scalar(64, c1, c1), -1);
                    }
                }
            }
        }
        cv::imshow("Intensity Image", intensityImage);

        cv::Mat distImageGray, distImageInv, distTransformedImg;
        cv::cvtColor(distanceImage, distImageGray, cv::COLOR_BGR2GRAY);
        cv::threshold(distImageGray, distImageInv, 1, 255.0, cv::THRESH_BINARY_INV);
        cv::distanceTransform(distImageInv, distTransformedImg, cv::DIST_L2, 3);

        condensation.updateSample();
        for (size_t i = 0; i < condensation.samples.size(); ++i) {
            cv::Point center(cvRound(condensation.samples[i].at<float>(0)), cvRound(condensation.samples[i].at<float>(1)));
            int angle = cvRound(condensation.samples[i].at<float>(2));
            condensation.confidence[i] = CalculateBodyLikelihood(distTransformedImg, center, angle);
        }
        condensation.updateByTime();

        cv::Point usrPos(cvRound(condensation.state.at<float>(0)), cvRound(condensation.state.at<float>(1)));
        int usrAngl = static_cast<int>(condensation.state.at<float>(2)) % 360;

        int aveIntensity = 0;
        cv::Rect roi(usrPos.x - 30, usrPos.y - 30, 60, 60);
        roi &= cv::Rect(0, 0, g_windowSize.width, g_windowSize.height); // [修正]
        if (roi.area() > 0) {
            cv::Mat intensityGray;
            cv::cvtColor(intensityImage, intensityGray, cv::COLOR_BGR2GRAY);
            cv::Mat roiImg = intensityGray(roi);
            int nonZero = cv::countNonZero(roiImg);
            aveIntensity = (nonZero > 0) ? (cv::sum(roiImg)[0] / nonZero) : 0;
        }

        if (isTracked && aveIntensity < stopTrackingIntensity) isTracked = false;

        if (!isTracked) {
             for (const auto& handler : handlerList) {
                for (size_t i = 0; i < handler.urgDistance.size(); ++i) {
                    if (handler.urgDistance[i] < startTrackingDistanceMax && handler.urgIntensity[i] > startTrackingIntensityMin) {
                        initValue.at<float>(0) = handler.drawPosition[i].x;
                        initValue.at<float>(1) = handler.drawPosition[i].y;
                        condensation.initSampleSet(initValue, initMean, initDeviation);
                        isTracked = true;
                        goto found_target_multi;
                    }
                }
            }
        }
        found_target_multi:;

        if (g_lbPressed) {
            g_lbPressed = false;
            initValue.at<float>(0) = (float)g_mouseClickPos.x;
            initValue.at<float>(1) = (float)g_mouseClickPos.y;
            condensation.initSampleSet(initValue, initMean, initDeviation);
            usrPos = g_mouseClickPos;
            isTracked = true;
        }

        if (isTracked) {
            DrawBodyContour(dispImage, usrPos, usrAngl);
            cv::rectangle(dispImage, roi, cv::Scalar(255, 255, 255), 1);
            cv::putText(dispImage, std::to_string(aveIntensity), usrPos + cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255));
            
            if (usrPos.x != 0 || usrPos.y != 0) {
                double img_x = usrPos.x - g_windowSize.width / 2.0; // [修正]
                double img_y = usrPos.y - g_windowSize.height / 2.0; // [修正]
                
                double robot_x_mm = -img_y / g_scale;
                double robot_y_mm = -img_x / g_scale;

                geometry_msgs::Pose2D msg;
                msg.x = robot_x_mm / 1000.0;
                msg.y = robot_y_mm / 1000.0;
                msg.theta = -usrAngl * M_PI_VAL / 180.0;
                pubPosePersonFollowing.publish(msg);
            }
        }

        cv::imshow("Display Image", dispImage);
        if (cv::waitKey(1) == 'q') break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
}
