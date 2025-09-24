// 1台のLiDARのmsgから人を追跡する

#include <vector>
#include <stdio.h>
#include <string.h>
#include <cmath> // FIX: 添加 cmath 头文件以使用 std::round
#include <opencv2/opencv.hpp>

#include "EllipseTracker.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

// ////////////////
// // Defines
// ////////////////

// #define M_PI 3.141592653589793238

constexpr int SCAN_STEP_NUM = 1080;
constexpr double SCAN_DISTANCE_MAGNIFICATION = 1000.0;
constexpr double SCAN_DISTANCE_DEFAULT = 30000;
constexpr double SCAN_INTENSITY_DEFAULT = 0.0;

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

// ////////////////
// // Functions
// ////////////////

int main(int argc, char **argv)
{
	// ---Configure---
	std::string defNodeName = "person_following";
	std::string defTopicNameScan = "scan";
	std::string defTopicNameTrackingPosition = "pose_person_following";
	double defOffsetPoseX = 0;
	double defOffsetPoseY = 0;
	double defOffsetPoseT = -0.5 * M_PI;

	double defStartTrackingIntensityMin = 80;	// 自動的にトラッキングを始める最小反射強度
	double defStartTrackingDistanceMax  = 2200; // 自動的にトラッキングを始める最大距離
	double defStopTrackingIntensity 	= 32;	// トラッキングをやめる強度
	double defRetroreflectiveIntensity  = 50000;// 再帰性反射材の強度 // 200000

	// 距離データ表示用設定
	const int DispImageWidth  = 640;
	const int DispImageHeight = 640;
	float Scale = 0.1f;		 // 距離対ピクセルのスケーリング（0.1のとき1cmが1ピクセルに対応）
	const double OffsetX = 0;   //ワールド座標系におけるLiDARの横位置[m]
	const double OffsetY = 0;   //ワールド座標系におけるLiDARの縦位置[m]
	const int OffsetC = DispImageWidth/2  + Scale*OffsetX; // 画像中でのセンサ位置のX軸方向オフセット
	const int OffsetR = DispImageHeight/2 - Scale*OffsetY; // 画像中でのセンサ位置のY軸方向オフセット
	float OffsetRotate = 0;		 // X（右手）軸に対するセンサの右回り回転角度[deg]
	float ThresL = 3000;	 // 遠距離のしきい値(3000=3m)
	float ThresM = 1000;	 // 中距離のしきい値(1000=1m)
	float ThresS = 200;		 // 近距離のしきい値(200=20cm)


	int URG_Distance[SCAN_STEP_NUM];
	int URG_Intensity[SCAN_STEP_NUM];
	int URG_Intensity_Raw[SCAN_STEP_NUM];

	int isTracked = 0;
	double aveIntensity = 0;

	// ---ROS Initialize---
	ros::init(argc, argv, defNodeName.c_str());
	ros::NodeHandle nodeHandle;

	ros::Subscriber subLaserScan = nodeHandle.subscribe<sensor_msgs::LaserScan>(
		defTopicNameScan.c_str(), 100,
		// FIX: Lambda 表达式使用 [&] 捕获外部变量
		[&](const sensor_msgs::LaserScan::ConstPtr &msg) {
			int sizeScan = msg->ranges.size();
			int indexStartScan = (SCAN_STEP_NUM - sizeScan) * 0.5;
			for (int i = 0; i < SCAN_STEP_NUM; i++)
			{
				URG_Distance[i] = SCAN_DISTANCE_DEFAULT;
				URG_Intensity_Raw[i] = SCAN_INTENSITY_DEFAULT;
			}
			for (int i = 0; i < sizeScan; i++)
			{
				if(i >= msg->ranges.size() || (indexStartScan + i) >= SCAN_STEP_NUM) continue;
				URG_Distance[indexStartScan + i] = msg->ranges[i] * SCAN_DISTANCE_MAGNIFICATION; //m -> mm
				URG_Intensity_Raw[indexStartScan + i] = msg->intensities[i];
			}
			return;
		});

	ros::Publisher pubPosePersonFollowing = nodeHandle.advertise<geometry_msgs::Pose2D>(defTopicNameTrackingPosition.c_str(), 100);
	geometry_msgs::Pose2D msgPosePersonFollowing;

	cv::Mat baseImage      (cv::Size(DispImageWidth, DispImageHeight), CV_8UC3);
	cv::Mat distanceImage  (cv::Size(DispImageWidth, DispImageHeight), CV_8UC3);
	cv::Mat intensityImage (cv::Size(DispImageWidth, DispImageHeight), CV_8UC1);

	// sin cosのテーブルを作成
	double cosVal[SCAN_STEP_NUM];
	double sinVal[SCAN_STEP_NUM];
	for (int ii = 0; ii < SCAN_STEP_NUM; ii++)
	{
		cosVal[ii] = cos((ii * 0.25 + OffsetRotate) * (M_PI / 180));
		sinVal[ii] = sin((ii * 0.25 + OffsetRotate) * (M_PI / 180));
	}

	// センサ位置と観測範囲の描画
	baseImage.setTo(cv::Scalar(0,0,0));
	// FIX: cv::round -> std::round, CV_RGB -> cv::Scalar
	cv::circle(baseImage, cv::Point(std::round(OffsetC), std::round(OffsetR)), 4, cv::Scalar(64, 64, 64), -1); // センサの位置
	for (int jj = 0; jj < SCAN_STEP_NUM; jj++)
	{
		cv::circle(baseImage, cv::Point(std::round(ThresS * Scale * sinVal[jj]) + OffsetC, std::round(ThresS * Scale * cosVal[jj]) + OffsetR), 1, cv::Scalar(64, 64, 64), -1); // 近距離のしきい値の円孤
		cv::circle(baseImage, cv::Point(std::round(ThresM * Scale * sinVal[jj]) + OffsetC, std::round(ThresM * Scale * cosVal[jj]) + OffsetR), 1, cv::Scalar(32, 32, 32), -1); // 中距離のしきい値の円弧
		cv::circle(baseImage, cv::Point(std::round(ThresL * Scale * sinVal[jj]) + OffsetC, std::round(ThresL * Scale * cosVal[jj]) + OffsetR), 1, cv::Scalar(64, 64, 64), -1); // 遠距離のしきい値の円弧
	}

	//**************************************************************************************************
	// OpenCVの準備
	//**************************************************************************************************
	// FIX: 修正 cv::Mat 的初始化语法
	cv::Mat dispImage(cv::Size(DispImageWidth, DispImageHeight), CV_8UC3);	   // 結果表示用画像
	cv::Mat distImage(cv::Size(DispImageWidth, DispImageHeight), CV_8UC1);	   // センサで取得した距離データを描画する画像
	cv::Mat distImageNot(cv::Size(DispImageWidth, DispImageHeight), CV_8UC1); // センサで取得した距離データを描画した画像を反転
	cv::Mat transImage(cv::Size(DispImageWidth, DispImageHeight), CV_32FC1);  // 距離画像に変換

	// ウインドウの準備
	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Intensity Image", cv::WINDOW_AUTOSIZE);

	// マウスコールバック関数の登録
	cv::setMouseCallback("Display Image", mouseCallback);

	// FIX: 清理了旧的、被注释掉的 C-Style OpenCV 代码
	EllipseTrackerPool trackers(1, 300, 24, 12, cv::Vec2d(OffsetC, OffsetR));

	//**************************************************************************************************
	// Main Loop
	//**************************************************************************************************
	ros::Rate loop_rate(40);
	while (ros::ok())
	{
		// ベース画像で初期化
		baseImage.copyTo(dispImage);
		baseImage.copyTo(distanceImage);
		intensityImage.setTo(cv::Scalar(0));

		// 反射強度正規化用一時変数
		int x[SCAN_STEP_NUM], y[SCAN_STEP_NUM];
		double URG_Distance_I[SCAN_STEP_NUM], URG_Intensity_I[SCAN_STEP_NUM];
		double angle[SCAN_STEP_NUM], a_1[SCAN_STEP_NUM], a_2[SCAN_STEP_NUM], b_1[SCAN_STEP_NUM], b_2[SCAN_STEP_NUM], c[SCAN_STEP_NUM];
		// 1個目のレーザセンサの反射強度正規化
		for (int i = 0; i < SCAN_STEP_NUM; i++)
		{
			// FIX: cv::round -> std::round
			x[i] = std::round(URG_Distance[i] * Scale * sinVal[i]) + OffsetC;
			y[i] = std::round(URG_Distance[i] * Scale * cosVal[i]) + OffsetR;
			
			//反射強度正規化(begin) 正規化反射強度:URG_Intensity_I
			if (i != 0)
			{
				a_1[i] = OffsetC - x[i];  //ベクトルaのx成分
				a_2[i] = OffsetR - y[i];  //ベクトルaのy成分
				b_1[i] = x[i - 1] - x[i]; //ベクトルbのx成分
				b_2[i] = y[i - 1] - y[i]; //ベクトルbのy成分
                double mag_a = sqrt((double)(a_1[i] * a_1[i] + a_2[i] * a_2[i]));
                double mag_b = sqrt((double)(b_1[i] * b_1[i] + b_2[i] * b_2[i]));
                if (mag_a > 1e-6 && mag_b > 1e-6) {
				    c[i] = (a_1[i] * b_1[i] + a_2[i] * b_2[i]) / (mag_a * mag_b);
                } else {
                    c[i] = 1.0;
                }
				angle[i] = sqrt(1 - (c[i] * c[i]));
			} else {
                angle[i] = 1.0;
            }
			
			URG_Distance_I[i] = pow((double)URG_Distance[i], (double)0.287);
            if (angle[i] > 1e-6) {
			    URG_Intensity_I[i] = URG_Intensity_Raw[i] * URG_Distance_I[i] / pow((double)angle[i], (double)0.196);
            } else {
                URG_Intensity_I[i] = URG_Intensity_Raw[i] * URG_Distance_I[i];
            }
			
			URG_Intensity_I[i] = URG_Intensity_Raw[i];
			URG_Intensity[i] = std::round(URG_Intensity_I[i] / defRetroreflectiveIntensity * 255);
			if (URG_Intensity[i] > 255)
				URG_Intensity[i] = 255;
			if (URG_Intensity[i] < 0)
				URG_Intensity[i] = 0;
			//正規化(end)
		}

		//1個目のセンサ情報を描画
		for (int i = 0; i < SCAN_STEP_NUM; i++)
		{
			if (URG_Distance[i] > ThresS && URG_Distance[i] < ThresL)
			{
				// FIX: cv::round -> std::round, CV_RGB -> cv::Scalar
				cv::circle(distanceImage,  cv::Point(std::round(URG_Distance[i] * Scale * sinVal[i]) + OffsetC, std::round(URG_Distance[i] * Scale * cosVal[i]) + OffsetR), 1, cv::Scalar(255, 255, 0), -1);
				cv::circle(intensityImage, cv::Point(std::round(URG_Distance[i] * Scale * sinVal[i]) + OffsetC, std::round(URG_Distance[i] * Scale * cosVal[i]) + OffsetR), 1, cv::Scalar(URG_Intensity[i]), -1);
				int color1 = URG_Intensity[i] * 4 + 64;
				if (color1 > 255) color1 = 255;
				cv::circle(dispImage, cv::Point(std::round(URG_Distance[i] * Scale * sinVal[i]) + OffsetC, std::round(URG_Distance[i] * Scale * cosVal[i]) + OffsetR), 1, cv::Scalar(color1, color1, 64), -1);
			}
		}

		cv::imshow("Intensity Image", intensityImage);
		/***************************************************************************************************************/

		cv::cvtColor(distanceImage, distImage, cv::COLOR_BGR2GRAY);
		cv::threshold(distImage, distImageNot, 64.0, 255.0, cv::THRESH_BINARY_INV);
		// FIX: cv::distTransform -> cv::distanceTransform
		cv::distanceTransform(distImageNot, transImage, cv::DIST_L2, 3);

		// サンプルの更新
		trackers.next(transImage);

		int usrU=-100;     // 人物位置のx座標
		int usrV=-100;     // 人物位置のy座標
		int usrAngl=-100;  // 人物胴体の角度[deg]
		// 追跡結果(人物位置)を他で使うために取り出す
		if(!trackers.empty()){
			auto _p = trackers[0]->getPos();
			usrU    = _p[0];
			usrV    = _p[1];
			usrAngl = _p[2];
		}

		//追跡器の周りの平均反射強度を計算
		if(!trackers.empty()){
			cv::Rect roi = cv::Rect(usrU - 30, usrV - 30, 60, 60) & cv::Rect(0, 0, intensityImage.cols, intensityImage.rows);
			if(roi.area() > 0) {
				cv::Mat intensityImageROI(intensityImage, roi);
				double sum = cv::sum(intensityImageROI)[0];
				int count = cv::countNonZero(intensityImageROI);
				aveIntensity = (count > 0) ? (sum / count) : 0.0;
			} else {
				aveIntensity = 0.0;
			}
		}
		else aveIntensity=0.0;

		//追跡器の周りの平均反射強度が低いときには追跡をやめる
		if (!trackers.empty() && aveIntensity < defStopTrackingIntensity)
		{
			trackers.removeByOrder(0);
		}

		//追跡していないときに，近くで高反射強度が検出された場合，初期値変更して再追跡
		if (trackers.empty())
		{ 
			for (int i = 0; i < SCAN_STEP_NUM; i++)
			{
				if (URG_Distance[i] < defStartTrackingDistanceMax && URG_Intensity[i] > defStartTrackingIntensityMin)
				{
					// FIX: cv::round -> std::round
					int initU = std::round((URG_Distance[i] * Scale * sinVal[i]) + OffsetC - 20);
					int initV = std::round((URG_Distance[i] * Scale * cosVal[i]) + OffsetR);
					trackers.add(initU, initV, 0);
					if(!trackers.empty()) {
						trackers.next(transImage);
						auto _p = trackers[0]->getPos();
						usrU = _p[0];
						usrV = _p[1];
						usrAngl = _p[2];
					}
					break;
				}
			}
		}

		//マウスでクリックされたらその位置を初期位置として追跡を開始
		if (lbPressed)
		{
			lbPressed = false; 
			usrU = lbX;
			usrV = lbY;
			if(trackers.empty()){
				trackers.add(usrU, usrV, 0); // 角度は0で初期化
			}
			else{
				trackers[0]->init(usrU, usrV, 0); // 角度は0で初期化
			}
            if(!trackers.empty()) {
			    trackers.next(transImage);
			    auto _p = trackers[0]->getPos();
			    usrU = _p[0];
			    usrV = _p[1];
			    usrAngl = _p[2];
            }
		}

		// 追跡結果の描画
		if (!trackers.empty())
		{
			// FIX: 拼写错误 dispimage -> dispImage, CV_RGB -> cv::Scalar
			cv::ellipse(dispImage, cv::Point(usrU, usrV), cv::Size(24,12), usrAngl, 0, 360, cv::Scalar(255,255,255), 2);
			cv::rectangle(dispImage, cv::Point(usrU - 30, usrV - 30), cv::Point(usrU + 30, usrV + 30), cv::Scalar(255, 255, 255), 1, 8, 0);
			if (aveIntensity < 0)
				aveIntensity = 0;
			char mytext[12];
			sprintf(mytext, "%4.0lf", aveIntensity);
			cv::putText(dispImage, mytext, cv::Point(usrU + 30, usrV + 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 255, 255), 1);
		}

		// For Follow Tracking
		if (!trackers.empty() && (usrU != 0 && usrV != 0))
		{
			double rotX = (usrU - OffsetC) * cos(defOffsetPoseT) - (usrV - OffsetR) * sin(defOffsetPoseT);
			double rotY = (usrU - OffsetC) * sin(defOffsetPoseT) + (usrV - OffsetR) * cos(defOffsetPoseT);
			msgPosePersonFollowing.x = defOffsetPoseX + (-1.0) * rotX / (SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.y = defOffsetPoseY + (-1.0) * rotY / (SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.theta = defOffsetPoseT  + (usrAngl * M_PI / 180.0); // FIX: 角度转换应该是 /180
			pubPosePersonFollowing.publish(msgPosePersonFollowing);
		}

		/***************************************************************************************************************/
		cv::imshow("Display Image", dispImage);

		if (cv::waitKey(1) == 'q')
		{
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	cv::destroyAllWindows();

	return 0;
}