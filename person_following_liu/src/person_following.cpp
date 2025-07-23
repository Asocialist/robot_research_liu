#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>

#include "MyEllipseNormalEvaluation.h"
#include "MyCondensation.h" // 修正済みのC++版ヘッダ
#include "json11.hpp"

// --- 定数 ---
constexpr double M_PI_VAL = 3.1415926535;
constexpr int SCAN_STEP_NUM = 1080;
constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;
constexpr double SCAN_DISTANCE_DEFAULT = 60.0 * SCAN_DISTANCE_MAGNIFICATION;
constexpr double SCAN_INTENSITY_DEFAULT = 0.0;

// --- グローバル変数 ---
std::vector<int> g_URG_Distance(SCAN_STEP_NUM, SCAN_DISTANCE_DEFAULT);
std::vector<int> g_URG_Intensity(SCAN_STEP_NUM, SCAN_INTENSITY_DEFAULT);
std::vector<int> g_URG_Intensity_Raw(SCAN_STEP_NUM, SCAN_INTENSITY_DEFAULT);

bool g_lbPressed = false;
cv::Point g_mouseClickPos;

// --- マウスコールバック関数 ---
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        g_lbPressed = true;
        g_mouseClickPos = cv::Point(x, y);
    }
}

// --- レーザースキャン コールバック関数 ---
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int sizeScan = msg->ranges.size();
    if (sizeScan == 0) return;
    int indexStartScan = (SCAN_STEP_NUM - sizeScan) / 2;

    std::fill(g_URG_Distance.begin(), g_URG_Distance.end(), SCAN_DISTANCE_DEFAULT);
    std::fill(g_URG_Intensity_Raw.begin(), g_URG_Intensity_Raw.end(), SCAN_INTENSITY_DEFAULT);

    for (int i = 0; i < sizeScan; ++i) {
        if (indexStartScan + i < SCAN_STEP_NUM) {
            g_URG_Distance[indexStartScan + i] = msg->ranges[i] * SCAN_DISTANCE_MAGNIFICATION;
            g_URG_Intensity_Raw[indexStartScan + i] = msg->intensities[i];
        }
    }
}


int main(int argc, char** argv) {
    // --- 設定項目 ---
    std::string nodeName = "person_following";
    std::string topicNameScan = "scan";
    std::string topicNameTrackingPosition = "pose_person_following";
    double offsetPoseX = 0.0;
    double offsetPoseY = 0.0;
    double offsetPoseT = -0.5 * M_PI_VAL;

    double startTrackingIntensityMin = 80.0;
    double startTrackingDistanceMax = 2200.0;
    double stopTrackingIntensity = 32.0;
    double retroreflectiveIntensity = 50000.0;

    // 距離データ表示用設定
    const int OffsetX = 320;
    const int OffsetY = 320;
    const float Scale = 0.1f;
    const float Rotate = 45.0f;
    const float ThresL = 3000.0f;
    const float ThresM = 1000.0f;
    const float ThresS = 200.0f;

    // --- ROSの初期化 ---
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nodeHandle;
    ros::Subscriber subLaserScan = nodeHandle.subscribe(topicNameScan, 100, laserScanCallback);
    ros::Publisher pubPosePersonFollowing = nodeHandle.advertise<geometry_msgs::Pose2D>(topicNameTrackingPosition, 100);

    // --- OpenCVの準備 ---
    cv::Mat baseImage = cv::Mat::zeros(640, 640, CV_8UC3);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Display Image", mouseCallback);

    // sin/cosテーブルの作成
    std::vector<double> cosVal(SCAN_STEP_NUM), sinVal(SCAN_STEP_NUM);
    for (int i = 0; i < SCAN_STEP_NUM; ++i) {
        double angleRad = (i * 0.25 + Rotate) * (M_PI_VAL / 180.0);
        cosVal[i] = cos(angleRad);
        sinVal[i] = sin(angleRad);
    }

    // ベース画像の描画
    cv::circle(baseImage, cv::Point(OffsetX, OffsetY), 4, cv::Scalar(64, 64, 64), -1, cv::LINE_AA);
    for (int i = 0; i < SCAN_STEP_NUM; ++i) {
        cv::circle(baseImage, cv::Point(cvRound(ThresS * Scale * sinVal[i]) + OffsetX, cvRound(ThresS * Scale * cosVal[i]) + OffsetY), 1, cv::Scalar(64, 64, 64), -1, cv::LINE_AA);
        cv::circle(baseImage, cv::Point(cvRound(ThresM * Scale * sinVal[i]) + OffsetX, cvRound(ThresM * Scale * cosVal[i]) + OffsetY), 1, cv::Scalar(32, 32, 32), -1, cv::LINE_AA);
        cv::circle(baseImage, cv::Point(cvRound(ThresL * Scale * sinVal[i]) + OffsetX, cvRound(ThresL * Scale * cosVal[i]) + OffsetY), 1, cv::Scalar(64, 64, 64), -1, cv::LINE_AA);
    }

    // --- パーティクルフィルタの準備 ---
    MyCondensation condensation(3, 300); // 状態(x, y, angle), サンプル数300
    cv::Mat initValue = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);
    cv::Mat initMean = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);
    cv::Mat initDeviation = (cv::Mat_<float>(3, 1) << 5.0f, 5.0f, 20.0f);
    
    condensation.initSampleSet(initValue, initMean, initDeviation);
    condensation.updateSample();
    
    // 評価関数の準備
    SetSensorPosition(cv::Point(OffsetX, OffsetY));
    StoreBodyContourPosition(24, 12, 10); // 肩: 長軸24, 短軸12
    StoreHeadContourPosition(12, 10);     // 頭: 半径12

    bool isTracked = false;
    int aveIntensity = 0;

    // --- メインループ ---
    ros::Rate loop_rate(40);
    while (ros::ok()) {
        cv::Mat dispImage, distanceImage, intensityImage;
        baseImage.copyTo(dispImage);
        baseImage.copyTo(distanceImage);
        intensityImage = cv::Mat::zeros(640, 640, CV_8UC3);

        // --- 反射強度の正規化と描画 ---
        // (元のコードの正規化ロジックは複雑で、依存関係が不明なため、簡略化・コメントアウトしています)
        // (ここでは強度を255段階にスケールするのみ)
        for (int i = 0; i < SCAN_STEP_NUM; ++i) {
            int intensity = cvRound((double)g_URG_Intensity_Raw[i] / retroreflectiveIntensity * 255.0);
            g_URG_Intensity[i] = std::max(0, std::min(255, intensity));
        }

        for (int i = 0; i < SCAN_STEP_NUM; ++i) {
            if (g_URG_Distance[i] > ThresS && g_URG_Distance[i] < ThresL) {
                cv::Point pt(
                    cvRound(g_URG_Distance[i] * Scale * sinVal[i]) + OffsetX,
                    cvRound(g_URG_Distance[i] * Scale * cosVal[i]) + OffsetY
                );
                cv::circle(distanceImage, pt, 1, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
                int c = g_URG_Intensity[i];
                cv::circle(intensityImage, pt, 1, cv::Scalar(c, c, c), -1, cv::LINE_AA);
                int c1 = std::min(255, c * 4 + 64);
                cv::circle(dispImage, pt, 1, cv::Scalar(64, c1, c1), -1, cv::LINE_AA);
            }
        }
        cv::imshow("Intensity Image", intensityImage);

        // --- 距離変換画像の作成 ---
        cv::Mat distImageGray, distImageInv, distTransformedImg;
        cv::cvtColor(distanceImage, distImageGray, cv::COLOR_BGR2GRAY);
        cv::threshold(distImageGray, distImageInv, 64.0, 255.0, cv::THRESH_BINARY_INV);
        cv::distanceTransform(distImageInv, distTransformedImg, cv::DIST_L2, 3);

        // --- パーティクルフィルタ処理 ---
        condensation.updateSample();

        for (int i = 0; i < condensation.samples.size(); ++i) {
            cv::Point center(
                cvRound(condensation.samples[i].at<float>(0)),
                cvRound(condensation.samples[i].at<float>(1))
            );
            int angle = cvRound(condensation.samples[i].at<float>(2));
            condensation.confidence[i] = CalculateBodyLikelihood(distTransformedImg, center, angle);
        }

        condensation.updateByTime();

        cv::Point usrPos(
            cvRound(condensation.state.at<float>(0)),
            cvRound(condensation.state.at<float>(1))
        );
        int usrAngl = cvRound(condensation.state.at<float>(2));

        // --- 追跡状態の管理 ---
        cv::Rect roi(usrPos.x - 30, usrPos.y - 30, 60, 60);
        roi &= cv::Rect(0, 0, 640, 640); // 画像範囲内に収める
        if (roi.area() > 0) {
            cv::Mat intensityGray;
            cv::cvtColor(intensityImage, intensityGray, cv::COLOR_BGR2GRAY);
            cv::Mat roiImg = intensityGray(roi);
            int nonZero = cv::countNonZero(roiImg);
            aveIntensity = (nonZero > 0) ? (cv::sum(roiImg)[0] / nonZero) : 0;
        } else {
            aveIntensity = 0;
        }

        if (isTracked && aveIntensity < stopTrackingIntensity) {
            isTracked = false;
        }

        if (!isTracked) {
            for (int i = 0; i < SCAN_STEP_NUM; ++i) {
                if (g_URG_Distance[i] < startTrackingDistanceMax && g_URG_Intensity[i] > startTrackingIntensityMin) {
                    initValue.at<float>(0) = cvRound(g_URG_Distance[i] * Scale * sinVal[i] + OffsetX - 20);
                    initValue.at<float>(1) = cvRound(g_URG_Distance[i] * Scale * cosVal[i] + OffsetY);
                    condensation.initSampleSet(initValue, initMean, initDeviation);
                    condensation.updateByTime();
                    isTracked = true;
                    break;
                }
            }
        }
        
        if (g_lbPressed) {
            g_lbPressed = false;
            initValue.at<float>(0) = (float)g_mouseClickPos.x;
            initValue.at<float>(1) = (float)g_mouseClickPos.y;
            condensation.initSampleSet(initValue, initMean, initDeviation);
            condensation.updateByTime();
            usrPos = g_mouseClickPos;
            isTracked = true;
        }

        // --- 結果の描画とPublish ---
        if (isTracked) {
            DrawBodyContour(dispImage, usrPos, usrAngl);
            cv::rectangle(dispImage, roi, cv::Scalar(255, 255, 255), 1);
            cv::putText(dispImage, std::to_string(aveIntensity), usrPos + cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255));

            if (usrPos.x != 0 && usrPos.y != 0) {
                double rotX = (usrPos.x - OffsetX) * cos(offsetPoseT) - (usrPos.y - OffsetY) * sin(offsetPoseT);
                double rotY = (usrPos.x - OffsetX) * sin(offsetPoseT) + (usrPos.y - OffsetY) * cos(offsetPoseT);
                
                geometry_msgs::Pose2D msg;
                msg.x = offsetPoseX + (-1.0) * rotX / (SCAN_DISTANCE_MAGNIFICATION * Scale);
                msg.y = offsetPoseY + (-1.0) * rotY / (SCAN_DISTANCE_MAGNIFICATION * Scale);
                msg.theta = offsetPoseT + (usrAngl * M_PI_VAL / 180.0);
                pubPosePersonFollowing.publish(msg);
            }
        }

        cv::imshow("Display Image", dispImage);

        if (cv::waitKey(1) == 'q') {
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
}
