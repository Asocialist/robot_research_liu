/* fixed by liu to correct coordinate and config issues */

#include <vector>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

#include <opencv2/opencv.hpp>

#include "MyEllipseNormalEvaluation_2LS.h"
#include "MyCondensation.h"
#include "json11.hpp"

#define M_PI 3.14159265358979

// ================ Global variables ================
bool lbPressed = false;
int lbX = 0, lbY = 0;

void mouseCallback(int event, int x, int y, int flag, void *param) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        lbPressed = true;
        lbX = x;
        lbY = y;
    }
}

int isTracked = 0;
int aveIntensity = 0;

float Scale = 0.075f;  // 原0.1
cv::Point windowSize(640,640);   // FIXED: 全局唯一

float ThresL = 3000;
float ThresM = 1000;
float ThresS = 100;

// ==================================================
// ScanMessageHandler
// ==================================================
class ScanMessageHandler {
protected:
    static constexpr int DEF_SCAN_STEP_MAX = 1080;
    static constexpr double DEF_ANGLE_OFFSET = M_PI;

public:
    static constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;
    static double RetroreflectiveIntensity;

    std::string topicName;
    double offsetX = 0, offsetY = 0, offsetT = 0;

    std::vector<int> urgDistance;
    std::vector<int> urgIntensity;
    std::vector<double> sinVal, cosVal;
    std::vector<cv::Point2d> drawPosition;

    void TopicCallbackFunction(const sensor_msgs::LaserScan::ConstPtr &msg){
        if(dataSize != msg->ranges.size()){
            urgDistance.resize(msg->ranges.size());
            urgIntensity.resize(msg->ranges.size());
            CreateTrigonometricTable(msg->angle_min, msg->angle_increment, msg->ranges.size());
            dataSize = msg->ranges.size();
        }
        for (int i = 0; i < msg->ranges.size(); i++) {
            urgDistance[i] = msg->ranges[i] * SCAN_DISTANCE_MAGNIFICATION;
            urgIntensity[i] = msg->intensities[i];
        }
        IntensityNormalization();
        CalcDrawPosition();
    }

    void SetOffset(double x, double y, double th){
        offsetX = x; offsetY = y; offsetT = th;
        dataSize = 0;
    }

protected:
    void CreateTrigonometricTable(double angle_min, double angle_increment, int num){
        sinVal.resize(num);
        cosVal.resize(num);
        for(int i = 0; i < num; i++){
            sinVal[i] = sin(angle_min + angle_increment*i + offsetT + DEF_ANGLE_OFFSET);
            cosVal[i] = cos(angle_min + angle_increment*i + offsetT + DEF_ANGLE_OFFSET);
        }
    }

    void IntensityNormalization(){
        int N = std::min((int)urgDistance.size(), (int)urgIntensity.size());
        for (int i = 0; i < N; i++){
            double val = urgIntensity[i] * pow((double)urgDistance[i], 0.5);
            urgIntensity[i] = std::clamp((int)(val / RetroreflectiveIntensity * 255), 0, 255);
        }
    }

    void CalcDrawPosition(){
        if(drawPosition.size() != dataSize) drawPosition.resize(dataSize);
        for (int i = 0; i < dataSize; i++){
            if (urgDistance[i] > ThresS && urgDistance[i] < ThresL){
                drawPosition[i].x = std::round(urgDistance[i] * Scale * sinVal[i]) 
                                    + windowSize.x*0.5 - offsetY * Scale;
                drawPosition[i].y = std::round(urgDistance[i] * Scale * cosVal[i]) 
                                    + windowSize.y*0.5 - offsetX * Scale;
            }
        }
    }

    int dataSize = 0;
};
double ScanMessageHandler::RetroreflectiveIntensity;

// ==================================================
// main
// ==================================================
int main(int argc, char **argv)
{
    using namespace cv;
    std::string NodeName = "person_following_multi";
    std::string TopicNameTrackingPosition = "pose_person_following";
    double OutputTopicOffsetPoseX = 0;
    double OutputTopicOffsetPoseY = 0;
    double OutputTopicOffsetPoseT = -0.5*M_PI;

    double StartTrackingIntensityMin = 80;
    double StartTrackingDistanceMax  = 2200;
    double StopTrackingIntensity     = 32;
    ScanMessageHandler::RetroreflectiveIntensity = 200000;

    std::vector<ScanMessageHandler> handlerList;
    handlerList.push_back(ScanMessageHandler());
    handlerList.back().SetOffset(0,0,0);
    handlerList.back().topicName = "scan";

    int OffsetX[2] = {0,0};   // FIXED: 初始化
    int OffsetY[2] = {0,0};
    double OffsetT[2] = {0,0};

    // ---Load JSON config---
    if (argc >= 2) {
        auto configure = json11::LoadJsonFile(argv[1]);
        if(configure != nullptr){
            NodeName                  = configure["NodeName"].is_null()?NodeName:configure["NodeName"].string_value();
            StartTrackingIntensityMin = configure["StartTrackingIntensityMin"].is_null()?StartTrackingIntensityMin:configure["StartTrackingIntensityMin"].number_value();
            StartTrackingDistanceMax  = configure["StartTrackingDistanceMax"].is_null()?StartTrackingDistanceMax:configure["StartTrackingDistanceMax"].number_value();
            StopTrackingIntensity     = configure["StopTrackingIntensity"].is_null()?StopTrackingIntensity:configure["StopTrackingIntensity"].number_value();
            TopicNameTrackingPosition = configure["TopicNamePublishPosition"].is_null()?TopicNameTrackingPosition:configure["TopicNamePublishPosition"].string_value();
            ScanMessageHandler::RetroreflectiveIntensity = configure["RetroreflectiveIntensity"].is_null()?ScanMessageHandler::RetroreflectiveIntensity:configure["RetroreflectiveIntensity"].number_value();

            windowSize.x = configure["ImageSize"]["x"].is_null()?windowSize.x:configure["ImageSize"]["x"].number_value();
            windowSize.y = configure["ImageSize"]["y"].is_null()?windowSize.y:configure["ImageSize"]["y"].number_value();

            // FIXED: 统一使用 OutputOffset
            OutputTopicOffsetPoseX = configure["OutputOffset"]["x"].is_null()?OutputTopicOffsetPoseX:configure["OutputOffset"]["x"].number_value();
            OutputTopicOffsetPoseY = configure["OutputOffset"]["y"].is_null()?OutputTopicOffsetPoseY:configure["OutputOffset"]["y"].number_value();
            OutputTopicOffsetPoseT = configure["OutputOffset"]["theta"].is_null()?OutputTopicOffsetPoseT:configure["OutputOffset"]["theta"].number_value() - 0.5*M_PI;

            ThresS = configure["ThresholdDistance"]["short"].is_null()?ThresS:configure["ThresholdDistance"]["short"].number_value();
            ThresM = configure["ThresholdDistance"]["medium"].is_null()?ThresM:configure["ThresholdDistance"]["medium"].number_value();
            ThresL = configure["ThresholdDistance"]["long"].is_null()?ThresL:configure["ThresholdDistance"]["long"].number_value();

            if(configure["ScanList"].is_array()){
                auto slist = configure["ScanList"].array_items();
                handlerList.clear();
                int i=0;
                for(const auto &itr : slist){
                    if(itr["Offset"]["x"].is_null()||itr["Offset"]["y"].is_null()||itr["Offset"]["theta"].is_null()||itr["TopicName"].is_null()){
                        continue;
                    }
                    OffsetX[i] = -itr["Offset"]["y"].number_value()*Scale + windowSize.x/2;
                    OffsetY[i] = -itr["Offset"]["x"].number_value()*Scale + windowSize.y/2;
                    OffsetT[i] = -itr["Offset"]["theta"].number_value();

                    handlerList.push_back(ScanMessageHandler());
                    handlerList.back().SetOffset(itr["Offset"]["x"].number_value(),
                                                 itr["Offset"]["y"].number_value(),
                                                 itr["Offset"]["theta"].number_value());
                    handlerList.back().topicName = itr["TopicName"].string_value();
                    ++i;
                }
            }
        }
    }

    const int OffsetX_ = windowSize.x/2;
    const int OffsetY_ = windowSize.y/2;

    // ========== ROS 初始化 ==========
    ros::init(argc, argv, NodeName.c_str());
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subs;
    for(auto &h : handlerList){
        subs.push_back(nh.subscribe(h.topicName,100,&ScanMessageHandler::TopicCallbackFunction,&h));
    }
    ros::Publisher pubPose = nh.advertise<geometry_msgs::Pose2D>(TopicNameTrackingPosition,100);
    geometry_msgs::Pose2D msg;

    // ========== OpenCV窗口 ==========
    namedWindow("Display Image", WINDOW_AUTOSIZE);
    setMouseCallback("Display Image", mouseCallback);

    // ========== Condensation 初始化 ==========
    MyCondensation *ConDens = myCreateConDensation(3,300);
    cv::Mat initValue=cv::Mat::zeros(3,1,CV_32F);
    cv::Mat initMean=cv::Mat::zeros(3,1,CV_32F);
    cv::Mat initDev=cv::Mat::zeros(3,1,CV_32F);
    initValue.at<float>(2)=1080.0f;
    initDev.at<float>(0)=5.0f;
    initDev.at<float>(1)=5.0f;
    initDev.at<float>(2)=20.0f;
    myConDensInitSampleSet(ConDens,initValue,initMean,initDev);
    myConDensUpdateSample(ConDens);

    // ========== 主循环 ==========
    int prevusrU = -1, prevusrV = -1, prevusrAngl = -1;
    cv::Mat baseImage(windowSize, CV_8UC3, cv::Scalar(0,0,0));

    ros::Rate loop_rate(60);
    while(ros::ok()){
        cv::Mat dispImage;
        baseImage.copyTo(dispImage);

        // === 更新粒子滤波 ===
        myConDensUpdateSample(ConDens);
        myConDensUpdateByTime(ConDens);

        int usrU = round(ConDens->state.at<float>(0));
        int usrV = round(ConDens->state.at<float>(1));
        int usrAngl = round(ConDens->state.at<float>(2));
        usrAngl = (usrAngl % 360 + 360) % 360;

        // === 鼠标初始化 ===
        if(lbPressed){
            lbPressed = false;
            initValue.at<float>(0) = (float)lbX;
            initValue.at<float>(1) = (float)lbY;
            myConDensInitSampleSet(ConDens,initValue,initMean,initDev);
            myConDensUpdateSample(ConDens);
            myConDensUpdateByTime(ConDens);
            usrU = lbX; usrV = lbY;
            isTracked = 1;
        }

        // === 绘制人物矩形 ===
        if(isTracked==1){
            cv::rectangle(dispImage, Point(usrU-30,usrV-30), Point(usrU+30,usrV+30), Scalar(255,255,255),1);
            char angleText[32];
            sprintf(angleText,"Ang:%3d",usrAngl);
            putText(dispImage,angleText,Point(usrU+30,usrV+50),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,255));
        }

        // === 发布Pose ===
        if(isTracked==1 && (usrU!=0 && usrV!=0)){
            double dx = usrU - OffsetX_;
            double dy = usrV - OffsetY_;
            double Xr = +dy/(ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION*Scale);
            double Yr = -dx/(ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION*Scale);
            double c=cos(OutputTopicOffsetPoseT), s=sin(OutputTopicOffsetPoseT);
            msg.x = OutputTopicOffsetPoseX + c*Xr - s*Yr;
            msg.y = OutputTopicOffsetPoseY + s*Xr + c*Yr;
            msg.theta = OutputTopicOffsetPoseT + (-usrAngl*M_PI/180.0);
            pubPose.publish(msg);
            ROS_INFO("Person pose: x=%.2f y=%.2f theta=%.2f", msg.x,msg.y,msg.theta);
        }

        if(isTracked==1){
            prevusrU=usrU; prevusrV=usrV; prevusrAngl=usrAngl;
        }

        imshow("Display Image",dispImage);
        if(waitKey(1)=='q') break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    myReleaseConDensation(&ConDens);
    cv::destroyAllWindows();
    return 0;
}
