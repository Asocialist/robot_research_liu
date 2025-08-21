/*changed by liu to fit opencv4  */

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
// ////////////////
// // Defines
// ////////////////

#define M_PI 3.14159265358979

// ////////////////
// // Variables
// ////////////////

// マウスイベントの取得
bool lbPressed = false;
int lbX = 0;
int lbY = 0;

// マウスコールバック関数
void mouseCallback(int event, int x, int y, int flag, void *param)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		lbPressed = true;
		lbX = x;
		lbY = y;
	}
	return;
}

// 追跡中フラグ
int isTracked = 0;
int aveIntensity = 0;

// 定数
float Scale = 0.075f;  //元0.1f
cv::Point windowSize;

float ThresL = 3000;	 // 遠距離のしきい値(3000=3m)
float ThresM = 1000;	 // 中距離のしきい値(1000=1m)
//float ThresS = 200;		 // 近距離のしきい値(200=20cm)
float ThresS = 100;		 // 近距離のしきい値(100=10cm)

class ScanMessageHandler
{
	protected: // 定数
	static constexpr int DEF_SCAN_STEP_MAX = 1080;
	static constexpr double DEF_ANGLE_OFFSET = M_PI;
	public:
	static constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;

	public: // 設定(グローバル)
	static double RetroreflectiveIntensity;

	public: // 設定(ローカル)
	std::string topicName;
	double offsetX = 0;
	double offsetY = 0;
	double offsetT = 0;

	public: // 取得データ
	std::vector<int> urgDistance;
	std::vector<int> urgIntensity;
	std::vector<int> urgIntensityRaw;
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
		// 反射強度正規化用一時変数
		int x[DEF_SCAN_STEP_MAX], y[DEF_SCAN_STEP_MAX];
		double URG_Intensity_I[DEF_SCAN_STEP_MAX];
		double angle[DEF_SCAN_STEP_MAX], a_1[DEF_SCAN_STEP_MAX], a_2[DEF_SCAN_STEP_MAX], b_1[DEF_SCAN_STEP_MAX], b_2[DEF_SCAN_STEP_MAX], c[DEF_SCAN_STEP_MAX];
		//レーザセンサの反射強度正規化
		int arrayMinimal = std::min({urgDistance.size(), urgIntensity.size()/*, DEF_SCAN_STEP_MAX*/});
		for (int i = 0; i < arrayMinimal; i++)
		{


			//前の点と張る面の法線ベクトルと、次の点と張る面の法線ベクトルの平均を取って点の法線ベクトルとする

			int bi=i>0?i-1:i+1;
			int ni=i<arrayMinimal-1?i+1:i-1;
			// //視点位置
			// cv::Vec2d op(0, 0);
			//観測点
			cv::Vec2d dp (urgDistance[i]  * sinVal[i],  urgDistance[i]  * cosVal[i]);
			//前の観測点
			cv::Vec2d bdp(urgDistance[bi] * sinVal[bi], urgDistance[bi] * cosVal[bi]);
			//次の観測点
			cv::Vec2d ndp(urgDistance[ni] * sinVal[ni], urgDistance[ni] * cosVal[ni]);
			//視線ベクトル
			//cv::Vec2d viewv = dp-op; viewv/=cv::norm(viewv);
			cv::Vec2d viewv = dp; viewv/=cv::norm(viewv);
			//法線ベクトル
			cv::Vec2d np;
			//2点間距離閾値
			const double DP_THRESH=200; //200mm

			if(bi==ni){
				//隣の点とだけ評価
				cv::Vec2d v=ndp-dp;
				double nl = cv::norm(v);
				if(nl>DP_THRESH){
					np = viewv; //遠すぎたら不採用
				}else{
					np=cv::Vec2d(-v[1],v[0]); //90度回転
				}
			}else{
				//前後合わせて評価
				cv::Vec2d bv = bdp-dp;
				double bnl = cv::norm(bv);
				cv::Vec2d nv = ndp-dp;
				double nnl = cv::norm(nv);
				if(bnl>DP_THRESH && nnl>DP_THRESH){
					np = viewv; //遠すぎたら不採用
				}else if(bnl>DP_THRESH){
					np=cv::Vec2d(-nv[1],nv[0]); //次だけ採用
				}else if(nnl>DP_THRESH){
					np=cv::Vec2d(-bv[1],bv[0]); //前だけ採用
				}else{
					np = cv::Vec2d(-nv[1],nv[0]) + cv::Vec2d(-bv[1],bv[0]);//両方採用
				}
			}
			np/=cv::norm(np); //正規化
			
			//法線ベクトルと視線ベクトルの内積からcos(入射角)計算
			angle[i] = np.dot(viewv);

			// URG_Intensity_I[i] = urgIntensity[i] * pow((double)urgDistance[i], (double)0.287) / pow((double)angle[i], (double)0.196); //反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			URG_Intensity_I[i] = urgIntensity[i] * pow((double)urgDistance[i], (double)0.5);//反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			//修正点r^0.287 -> r^0.5
			urgIntensity[i] = std::round (URG_Intensity_I[i] / RetroreflectiveIntensity * 255);
			if (urgIntensity[i] > 255)
				urgIntensity[i] = 255;
			if (urgIntensity[i] < 0)
				urgIntensity[i] = 0;

		}
	}

	void CalcDrawPosition()
	{
		if(drawPosition.size() != dataSize){
			drawPosition.resize(dataSize);
		}
		int arrayMinimal = std::min({urgDistance.size()});
		for (int i = 0; i < arrayMinimal; i++)
		{
			if (urgDistance[i] > ThresS && urgDistance[i] < ThresL) //コメント文だった
			{
				drawPosition[i].x = std::round (urgDistance[i] * Scale * sinVal[i]) + windowSize.x * 0.5 - offsetY * Scale;
    			drawPosition[i].y = std::round (urgDistance[i] * Scale * cosVal[i]) + windowSize.y * 0.5 - offsetX * Scale;
			}
		}
	}

	protected: // ローカル変数
	int dataSize = 0;
};
double ScanMessageHandler::RetroreflectiveIntensity;


// ////////////////
// // Functions
// ////////////////

int main(int argc, char **argv)
{
	using namespace cv;
    
	// ---Configure---
	std::string NodeName = "person_following_multi";
	std::string TopicNameTrackingPosition = "pose_person_following";
	double OutputTopicOffsetPoseX = 0;
	double OutputTopicOffsetPoseY = 0;
	double OutputTopicOffsetPoseT = -0.5 * M_PI;

	double StartTrackingIntensityMin = 80;
	double StartTrackingDistanceMax  = 2200;
	double StopTrackingIntensity 	= 32;
	ScanMessageHandler::RetroreflectiveIntensity  = 200000;

	Point windowSize; // 使用 cv::Point
	windowSize = Point(640, 640);

	std::vector<ScanMessageHandler> handlerList;
	handlerList.push_back(ScanMessageHandler());
	handlerList.back().SetOffset(0, 0, 0);
	handlerList.back().topicName = "scan";

	int OffsetX[2];
	int OffsetY[2];
	double OffsetT[2];

	// ---Configure--- (JSON加载逻辑不变)
	{// ---Load configure---
	    if (argc >= 2) {
	        auto configure = json11::LoadJsonFile(argv[1]);
	        if (configure == nullptr) {
	            std::cerr << "cannnoot read configure file." << std::endl;
	        }
	        else{
	            NodeName                     = configure["NodeName"].is_null() ? NodeName : configure["NodeName"].string_value();
				StartTrackingIntensityMin    = configure["StartTrackingIntensityMin"].is_null() ? StartTrackingIntensityMin : configure["StartTrackingIntensityMin"].number_value();
	            StartTrackingDistanceMax     = configure["StartTrackingDistanceMax"].is_null() ? StartTrackingDistanceMax : configure["StartTrackingDistanceMax"].number_value();
	            StopTrackingIntensity        = configure["StopTrackingIntensity"].is_null() ? StopTrackingIntensity : configure["StopTrackingIntensity"].number_value();
				TopicNameTrackingPosition    = configure["TopicNamePublishPosition"].is_null() ? TopicNameTrackingPosition : configure["TopicNamePublishPosition"].string_value();
	            ScanMessageHandler::RetroreflectiveIntensity = configure["RetroreflectiveIntensity"].is_null() ? ScanMessageHandler::RetroreflectiveIntensity : configure["RetroreflectiveIntensity"].number_value();
	            
				windowSize.x = configure["ImageSize"]["x"].is_null() ? windowSize.x : configure["ImageSize"]["x"].number_value();
				windowSize.y = configure["ImageSize"]["y"].is_null() ? windowSize.y : configure["ImageSize"]["y"].number_value();
				
				OutputTopicOffsetPoseX = configure["OutputOffset"]["x"].is_null() ? OutputTopicOffsetPoseX : configure["GlobalOffset"]["x"].number_value();
				OutputTopicOffsetPoseY = configure["OutputOffset"]["y"].is_null() ? OutputTopicOffsetPoseY : configure["GlobalOffset"]["y"].number_value();
				OutputTopicOffsetPoseT = configure["OutputOffset"]["theta"].is_null() ? OutputTopicOffsetPoseT : configure["GlobalOffset"]["theta"].number_value() - (0.5 * M_PI);

				ThresS = configure["ThresholdDistance"]["short"].is_null() ? windowSize.x : configure["ThresholdDistance"]["short"].number_value();
				ThresM = configure["ThresholdDistance"]["medium"].is_null() ? windowSize.y : configure["ThresholdDistance"]["medium"].number_value();
				ThresL = configure["ThresholdDistance"]["long"].is_null() ? windowSize.x : configure["ThresholdDistance"]["long"].number_value();

				if(configure["ScanList"].is_array()){
					auto slist = configure["ScanList"].array_items();
					if(slist.size() != 0){
						handlerList.clear();
						int i = 0;
						for(const auto &itr : slist){
							if(itr["Offset"]["x"].is_null() || itr["Offset"]["y"].is_null() || itr["Offset"]["theta"].is_null() || itr["TopicName"].is_null()){
								continue;
							}
							OffsetX[i] = -itr["Offset"]["y"].number_value() * Scale + windowSize.x/2;
							OffsetY[i] = -itr["Offset"]["x"].number_value() * Scale + windowSize.y/2;
							OffsetT[i] = -itr["Offset"]["theta"].number_value();

							handlerList.push_back(ScanMessageHandler());
							handlerList.back().SetOffset(itr["Offset"]["x"].number_value(), itr["Offset"]["y"].number_value(), itr["Offset"]["theta"].number_value());
							handlerList.back().topicName = itr["TopicName"].string_value();

							++i;
						}
					}
				}
	        }

	    }
	    else {
	        //     std::cerr << "invalid argument." << std::endl;
	        //     exit(1);
	    }
	}// ---Load configure---

	const int OffsetX_ = windowSize.x * 0.5;
	const int OffsetY_ = windowSize.y * 0.5;

    // 创建 sensorVisibleRange
	Mat sensorVisibleRange[2] = {
        Mat(Size(windowSize.x, windowSize.y), CV_8UC1),
        Mat(Size(windowSize.x, windowSize.y), CV_8UC1)
    };
	for(int i=0;i<2;++i){
		sensorVisibleRange[i].setTo(Scalar(0));
		double oft = OffsetT[i]+M_PI*0.5;
		Vec2d ofv(cos(oft),sin(oft));
		double costh=cos(0.25*M_PI);
		for(int r=0;r<sensorVisibleRange[i].rows;++r){
		for(int c=0;c<sensorVisibleRange[i].cols;++c){
			Vec2d pv(c-OffsetX[i], r-OffsetY[i]);
			double co=pv.dot(ofv)/cv::norm(pv);
			sensorVisibleRange[i].at<uchar>(r, c) = (co<costh)?255:0;
		}}
	}
	{
		Mat tempNot;
        bitwise_not(sensorVisibleRange[1], tempNot);
        bitwise_and(sensorVisibleRange[0], tempNot, sensorVisibleRange[0]);
	}

	// ---ROS Initialize---
	ros::init(argc, argv, NodeName.c_str());
	ros::NodeHandle nodeHandle;
	std::vector<ros::Subscriber> subscLaserScan;
	for(int i = 0; i < handlerList.size(); i++){
		subscLaserScan.push_back(nodeHandle.subscribe(handlerList[i].topicName.c_str(), 100, &ScanMessageHandler::TopicCallbackFunction, &(handlerList[i])));
	}
	ros::Publisher pubPosePersonFollowing = nodeHandle.advertise<geometry_msgs::Pose2D>(TopicNameTrackingPosition.c_str(), 100);
	geometry_msgs::Pose2D msgPosePersonFollowing;

    // ================== 使用 cv::Mat 声明所有图像 ==================
    Size imageSize(windowSize.x, windowSize.y);
	Mat baseImage(imageSize, CV_8UC3, Scalar(0, 0, 0));
	Mat distanceImage(imageSize, CV_8UC3, Scalar(0, 0, 0));
	Mat intensityImage(imageSize, CV_8UC3, Scalar(0, 0, 0));
	Mat maskImage(imageSize, CV_8UC1, Scalar(0));
	Mat maskImageBinInv(imageSize, CV_8UC1, Scalar(0));
	Mat distanceImage2(imageSize, CV_8UC3, Scalar(0, 0, 0));
	Mat intensityImage2(imageSize, CV_8UC3, Scalar(0, 0, 0));
    Mat dispImage(imageSize, CV_8UC3);
	Mat distImage(imageSize, CV_8UC1);
	Mat distImageNot(imageSize, CV_8UC1);
	Mat transImage(imageSize, CV_32F);
	Mat dispImage2(imageSize, CV_8UC3);

	// センサ位置と観測範囲の描画
	constexpr int SCAN_STEP_NUM = 1080;
	const float Rotate = 45;
	circle(baseImage, Point(round(OffsetX_), round(OffsetY_)), 4, Scalar(64, 64, 64), -1, 8);
	for (int jj = 0; jj < SCAN_STEP_NUM; jj++)
	{
		double angle = (jj * 0.25 + Rotate) * (M_PI / 180);
		circle(baseImage, Point(round(ThresS * Scale * sin(angle)) + OffsetX_, round(ThresS * Scale * cos(angle)) + OffsetY_), 1, Scalar(64, 64, 64), -1, 8);
		circle(baseImage, Point(round(ThresM * Scale * sin(angle)) + OffsetX_, round(ThresM * Scale * cos(angle)) + OffsetY_), 1, Scalar(32, 32, 32), -1, 8);
		circle(baseImage, Point(round(ThresL * Scale * sin(angle)) + OffsetX_, round(ThresL * Scale * cos(angle)) + OffsetY_), 1, Scalar(64, 64, 64), -1, 8);
	}

	// ================== OpenCV 窗口和鼠标设置 ==================
	namedWindow("Display Image", WINDOW_AUTOSIZE);
	setMouseCallback("Display Image", mouseCallback);

	// ================== 粒子滤波器准备 (现代C++版本) ==================
	MyCondensation *ConDens = myCreateConDensation(3, 300);
	Mat initValue = Mat::zeros(3, 1, CV_32F);
	Mat initMean = Mat::zeros(3, 1, CV_32F);
	Mat initDeviation = Mat::zeros(3, 1, CV_32F);

	initValue.at<float>(2) = 1080.0f; // 只有角度有初始值
	
	initDeviation.at<float>(0) = 5.0f;
	initDeviation.at<float>(1) = 5.0f;
	initDeviation.at<float>(2) = 20.0f;
	
	myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
	myConDensUpdateSample(ConDens);
	
    // **重要**: 以下自定义函数的签名必须在.h文件中被修改
	SetSensorPosition_2LS(0, Point(OffsetX[0], OffsetY[0]));
	SetSensorPosition_2LS(1, Point(OffsetX[1], OffsetY[1]));
	SetSensorVisibleRange_2LS(0, sensorVisibleRange[0]);
	SetSensorVisibleRange_2LS(1, sensorVisibleRange[1]);
	StoreBodyContourPosition_2LS(18, 11, 10);
	StoreHeadContourPosition_2LS(9, 10);
	int prevusrU = -1;
	int prevusrV = -1;
	int prevusrAngl = -1;

	// ================== 主循环 ==================
	ros::Rate loop_rate(60);
	while (ros::ok())
	{
		baseImage.copyTo(dispImage);
		baseImage.copyTo(distanceImage);
		intensityImage.setTo(Scalar(0,0,0));
		maskImage.setTo(Scalar(0));
		maskImageBinInv.setTo(Scalar(0));
		baseImage.copyTo(dispImage2);
		baseImage.copyTo(distanceImage2);
		intensityImage2.setTo(Scalar(0,0,0));
		
		int loopNumber = 1;
		for(const auto &itr : handlerList){
			for (int i = 0; i < itr.drawPosition.size(); i++)
			{
				if (itr.urgDistance[i] > ThresS && itr.urgDistance[i] < ThresL && loopNumber == 1)
				{
					circle(distanceImage, itr.drawPosition[i], 1, Scalar(255, 255, 0), -1, 8);
					circle(intensityImage, itr.drawPosition[i], 1, Scalar(itr.urgIntensity[i], itr.urgIntensity[i], itr.urgIntensity[i]), -1, 8);
					int color1 = itr.urgIntensity[i] * 4 + 64;
					circle(dispImage, itr.drawPosition[i], 1, Scalar(color1, color1, 64), -1, 8);
				}
				if (itr.urgDistance[i] > ThresS && itr.urgDistance[i] < ThresL && loopNumber == 2)
				{
					circle(distanceImage2, itr.drawPosition[i], 1, Scalar(255, 255, 0), -1, 8);
					circle(intensityImage2, itr.drawPosition[i], 1, Scalar(itr.urgIntensity[i], itr.urgIntensity[i], itr.urgIntensity[i]), -1, 8);
					int color1 = itr.urgIntensity[i] * 4 + 64;
					circle(dispImage2, itr.drawPosition[i], 1, Scalar(color1, color1, 64), -1, 8);
				}
				if( loopNumber == 1 && (i == 0 || i == (itr.drawPosition.size() - 1)) )
				{
					std::vector<Point> pPolyPoints1(4);
					std::vector<Point> pPolyPoints2(4);
					if(i == 0 && itr.offsetY > 0)
					{
						double kouten_x1 = -itr.drawPosition[i].y * ( ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) / ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) ) + itr.drawPosition[i].x;
						pPolyPoints1[0] = Point(0,0);
						pPolyPoints1[1] = Point(0,windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints1[2] = Point(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints1[3] = Point(kouten_x1,0);
						fillConvexPoly(maskImage, pPolyPoints1, Scalar(255), 8, 0);
					}
					if(i == (itr.drawPosition.size() - 1) && itr.offsetY > 0)
					{
						double kouten_x2 = ( ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) / ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) ) * (windowSize.y - itr.drawPosition[i].y) + itr.drawPosition[i].x;
						pPolyPoints2[0] = Point(0, windowSize.y);
						pPolyPoints2[1] = Point(0, windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints2[2] = Point(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints2[3] = Point(kouten_x2, windowSize.y);
						fillConvexPoly(maskImage, pPolyPoints2, Scalar(255), 8, 0);
					}
					threshold(maskImage, maskImageBinInv, 127, 255, THRESH_BINARY_INV);
				}
			}
			loopNumber++;
		}
		
		add(distanceImage, distanceImage2, distanceImage, maskImageBinInv);
		add(intensityImage, intensityImage2, intensityImage, maskImageBinInv);
		add(dispImage, dispImage2, dispImage, maskImageBinInv);

		imshow("Intensity Image", intensityImage);
		
		cvtColor(distanceImage, distImage, COLOR_BGR2GRAY);
		threshold(distImage, distImageNot, 64.0, 255.0, THRESH_BINARY_INV);
		distanceTransform(distImageNot, transImage, DIST_L2, 3);
		
		myConDensUpdateSample(ConDens);

		for (int i = 0; i < ConDens->samples_num; ++i)
		{
			ConDens->confidence.at<float>(0, i) = CalculateBodyLikelihood_2LS(transImage, 
                Point(round(ConDens->samples[i].at<float>(0)), round(ConDens->samples[i].at<float>(1))), 
                round(ConDens->samples[i].at<float>(2)));
		}

		myConDensUpdateByTime(ConDens);

		int usrU = round(ConDens->state.at<float>(0));
		int usrV = round(ConDens->state.at<float>(1));
		int usrAngl = round(ConDens->state.at<float>(2));
		usrAngl = (usrAngl % 360 + 360) % 360;

		cvtColor(intensityImage, distImage, COLOR_BGR2GRAY);
		Rect roi_rect(usrU - 30, usrV - 30, 60, 60);
        roi_rect &= Rect(0, 0, distImage.cols, distImage.rows); 
        if(roi_rect.area() > 0) {
            Mat roi = distImage(roi_rect);
            int nonZeroCount = countNonZero(roi);
            aveIntensity = (nonZeroCount > 0) ? (int)(sum(roi)[0] / nonZeroCount) : 0;
        } else {
            aveIntensity = 0;
        }

		if (isTracked == 1 && aveIntensity < StopTrackingIntensity)
		{
			initValue.at<float>(0) = 0.0f;
			initValue.at<float>(1) = 0.0f;
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
			myConDensUpdateSample(ConDens);
			myConDensUpdateByTime(ConDens);
			isTracked = 0;
		}

		if (isTracked == 0)
		{
			for(const auto &itr : handlerList){
				for (int i = 0; i < itr.urgDistance.size(); i++)
				{
					if (itr.urgDistance[i] < StartTrackingDistanceMax && itr.urgIntensity[i] > StartTrackingIntensityMin)
					{
						initValue.at<float>(0) = round((itr.urgDistance[i] * Scale * itr.sinVal[i]) + OffsetX_ - 20);
						initValue.at<float>(1) = round((itr.urgDistance[i] * Scale * itr.cosVal[i]) + OffsetY_);
						initValue.at<float>(2) = (float)prevusrAngl;
						myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
						myConDensUpdateSample(ConDens);
						myConDensUpdateByTime(ConDens);
						isTracked = 1;
						break;
					}
				}
				if(isTracked){
					break;
				}
			}
		}

		if (lbPressed)
		{
			lbPressed = false;
			initValue.at<float>(0) = (float)lbX;
			initValue.at<float>(1) = (float)lbY;
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
			myConDensUpdateSample(ConDens);
			myConDensUpdateByTime(ConDens);
			usrU = lbX;
			usrV = lbY;
			isTracked = 1;
		}

		if (isTracked == 1)
		{
			DrawBodyContour_2LS(dispImage, Point(usrU, usrV), usrAngl);
			rectangle(dispImage, Point(usrU - 30, usrV - 30), Point(usrU + 30, usrV + 30), Scalar(255, 255, 255), 1, 8, 0);
			if (aveIntensity < 0) aveIntensity = 0;
			char mytext[12];
			sprintf(mytext, "%4d", aveIntensity);
			putText(dispImage, mytext, Point(usrU + 30, usrV + 30), FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(255, 255, 255));
		
			char angleText[12];
			sprintf(angleText, "Ang: %3d", usrAngl);
			putText(dispImage, angleText, Point(usrU + 30, usrV + 50), FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(255, 255, 255));
		}

		if (isTracked == 1 && (usrU != 0 && usrV != 0))
		{
			double rotX = (usrU - OffsetX_) * cos(OutputTopicOffsetPoseT) - (usrV - OffsetY_) * sin(OutputTopicOffsetPoseT);
			double rotY = (usrU - OffsetX_) * sin(OutputTopicOffsetPoseT) + (usrV - OffsetY_) * cos(OutputTopicOffsetPoseT);
			msgPosePersonFollowing.x = OutputTopicOffsetPoseX + (-1.0) * rotX / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.y = OutputTopicOffsetPoseY - (-1.0) * rotY / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.theta = OutputTopicOffsetPoseT  + (-usrAngl*M_PI/180.0 + M_PI*1.5);
			pubPosePersonFollowing.publish(msgPosePersonFollowing);
			ROS_WARN("angle= %3d", usrAngl);
			ROS_INFO("msg=%f", msgPosePersonFollowing.theta);
		}

		if(isTracked == 1){
			prevusrU=usrU;
			prevusrV=usrV;
			prevusrAngl=usrAngl;
	    }

		imshow("Display Image", dispImage);

		if (waitKey(1) == 'q')
		{
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	// 内存由 cv::Mat 和 MyCondensation 析构函数自动管理
    myReleaseConDensation(&ConDens);
	destroyAllWindows();

	return 0;
}
