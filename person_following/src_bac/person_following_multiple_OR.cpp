/*
Sasaki Tomoki が改造したバージョン
2sensor の計測範囲の重なり考慮
line_350周辺

line_170周辺コメントアウト文復活
line_61 スケールを小さく
line_350周辺追跡の楕円サイズ修正
*/


#include <vector>
#include <stdio.h>
#include <string.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

//#include "MyEllipseNormalEvaluation.h"
#include "MyEllipseNormalEvaluation_2LS.h"
#include "MyCondensation.h"

#include "ros/ros.h"
#include "json11.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

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
	if (event == CV_EVENT_LBUTTONDOWN)
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
			// x[i] = cvRound(urgDistance[i] * Scale * sinVal[i]);
			// y[i] = cvRound(urgDistance[i] * Scale * cosVal[i]);
			// //反射強度正規化(begin) 正規化反射強度:URG_Intensity_I
			// if (i != 0)
			// {
			// 	a_1[i] = - x[i];  //ベクトルaのx成分
			// 	a_2[i] = - y[i];  //ベクトルaのy成分
			// 	b_1[i] = x[i - 1] - x[i]; //ベクトルbのx成分
			// 	b_2[i] = y[i - 1] - y[i]; //ベクトルbのy成分
			// 	c[i] = a_1[i] * b_1[i] + a_2[i] * b_2[i] /
			// 		sqrt((double)(a_1[i] * a_1[i] + a_2[i] * a_2[i])) * sqrt((double)(b_1[i] * b_1[i] + b_2[i] * b_2[i])); //2ベクトルのなす角からcos(90-x)を求める
			// 	angle[i] = sqrt(1 - (c[i] * c[i]));																		   //表面の法線ベクトルに対する入射角 sin^2 + cos^2 = 1よりsinを求め cosx = sin(90-x)より cosxをもとめた
			// }
			// // URG_Intensity_I[i] = urgIntensity[i] * pow((double)urgDistance[i], (double)0.287) / pow((double)angle[i], (double)0.196); //反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			// URG_Intensity_I[i] = urgIntensity[i] * pow((double)urgDistance[i], (double)0.5); //反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			// //修正点r^0.287 -> r^0.5
			// urgIntensity[i] = cvRound(URG_Intensity_I[i] / RetroreflectiveIntensity * 255);
			// if (urgIntensity[i] > 255)
			// 	urgIntensity[i] = 255;
			// if (urgIntensity[i] < 0)
			// 	urgIntensity[i] = 0;


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
			URG_Intensity_I[i] = urgIntensity[i] * pow((double)urgDistance[i], (double)0.5); //反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			//修正点r^0.287 -> r^0.5
			urgIntensity[i] = cvRound(URG_Intensity_I[i] / RetroreflectiveIntensity * 255);
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
				drawPosition[i].x = cvRound(urgDistance[i] * Scale * sinVal[i]) + windowSize.x * 0.5 - offsetY * Scale;
				drawPosition[i].y = cvRound(urgDistance[i] * Scale * cosVal[i]) + windowSize.y * 0.5 - offsetX * Scale;
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
	// ---Configure---
	std::string NodeName = "person_following_multi";
	std::string TopicNameTrackingPosition = "pose_person_following";
	double OutputTopicOffsetPoseX = 0;
	double OutputTopicOffsetPoseY = 0;
	double OutputTopicOffsetPoseT = -0.5 * M_PI;

	double StartTrackingIntensityMin = 80;	// 自動的にトラッキングを始める最小反射強度
	double StartTrackingDistanceMax  = 2200; // 自動的にトラッキングを始める最大距離
	double StopTrackingIntensity 	= 32;	// トラッキングをやめる強度 
	ScanMessageHandler::RetroreflectiveIntensity  = 200000;// 再帰性反射材の強度

	windowSize = cv::Point(640, 640);

	std::vector<ScanMessageHandler> handlerList;
	handlerList.push_back(ScanMessageHandler());
	handlerList.back().SetOffset(0, 0, 0);
	handlerList.back().topicName = "scan";

	int OffsetX[2]; // 画像中でのセンサ位置のX軸方向オフセット
	int OffsetY[2]; // 画像中でのセンサ位置のY軸方向オフセット
	double OffsetT[2];

	// ---Configure---
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

	// 定数
	// const int OffsetX = windowSize.x * 0.5; // 画像中でのセンサ位置のX軸方向オフセット
	// const int OffsetY = windowSize.y * 0.5; // 画像中でのセンサ位置のY軸方向オフセット

	const int OffsetX_ = windowSize.x * 0.5; // 画像中でのセンサ位置のX軸方向オフセット
	const int OffsetY_ = windowSize.y * 0.5; // 画像中でのセンサ位置のY軸方向オフセット

	//視野範囲画像の描画
	IplImage *sensorVisibleRange[2];
	for(int i=0;i<2;++i){
		sensorVisibleRange[i]=cvCreateImage(cvSize(windowSize.x, windowSize.y),IPL_DEPTH_8U,1);
		cvSet(sensorVisibleRange[i], cvScalar(0));
		double oft = OffsetT[i]+M_PI*0.5;
		cv::Vec2d ofv(cos(oft),sin(oft));
		double costh=cos(0.25*M_PI);
		for(int r=0;r<sensorVisibleRange[i]->height;++r){
		for(int c=0;c<sensorVisibleRange[i]->width;++c){
			cv::Vec2d pv(c-OffsetX[i], r-OffsetY[i]);
			double co=pv.dot(ofv)/cv::norm(pv);
			sensorVisibleRange[i]->imageData[c+r*sensorVisibleRange[i]->widthStep] = (co<costh)?255:0;
		}}
	}
	{
		IplImage *___t=cvCreateImage(cvSize(sensorVisibleRange[0]->width,sensorVisibleRange[0]->height),IPL_DEPTH_8U,1);
		cvNot(sensorVisibleRange[1], ___t);
		cvAnd(sensorVisibleRange[0], ___t, sensorVisibleRange[0]);
		cvReleaseImage(&___t);
	}

	cvShowImage("vis0",sensorVisibleRange[0]);
	cvShowImage("vis1",sensorVisibleRange[1]);
	cvWaitKey(0);
	cvDestroyAllWindows();


	// ---ROS Initialize---
	ros::init(argc, argv, NodeName.c_str());
	ros::NodeHandle nodeHandle;

	std::vector<ros::Subscriber> subscLaserScan;
	for(int i = 0; i < handlerList.size(); i++){
		subscLaserScan.push_back(nodeHandle.subscribe(handlerList[i].topicName.c_str(), 100, &ScanMessageHandler::TopicCallbackFunction, &(handlerList[i])));
	}
	
	ros::Publisher pubPosePersonFollowing = nodeHandle.advertise<geometry_msgs::Pose2D>(TopicNameTrackingPosition.c_str(), 100);
	geometry_msgs::Pose2D msgPosePersonFollowing;

	// 距離データ表示用設定
	IplImage *baseImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3);		 // 分かりやすくするためにセンサ位置や観測範囲などを描画しておくベース画像
	IplImage *distanceImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3);	 // センサで取得した距離データを描画する画像
	IplImage *intensityImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3); // 反射強度用画像

	IplImage *maskImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 1);		 // 片方のセンサの観測範囲を塗りつぶす画像
	IplImage *maskImageBin = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 1);		 // 片方のセンサの観測範囲を塗りつぶす二値画像
	IplImage *maskImageBinInv = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 1);		 // 片方のセンサの観測範囲を塗りつぶす二値画像の反転

	IplImage *distanceImage2 = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3);	 // 2つ目のセンサのデータ(マスク済み)
	IplImage *intensityImage2 = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3); // 同上


	// センサ位置と観測範囲の描画
	constexpr int SCAN_STEP_NUM = 1080;
	const float Rotate = 45;		 // 回転角度
	cvSetZero(baseImage);
	cvCircle(baseImage, cvPoint(cvRound(OffsetX_), cvRound(OffsetY_)), 4, CV_RGB(64, 64, 64), -1, 8); // センサの位置
	for (int jj = 0; jj < SCAN_STEP_NUM; jj++)
	{
		double angle = (jj * 0.25 + Rotate) * (M_PI / 180);
		cvCircle(baseImage, cvPoint(cvRound(ThresS * Scale * sin(angle)) + OffsetX_, cvRound(ThresS * Scale * cos(angle)) + OffsetY_), 1, CV_RGB(64, 64, 64), -1, 8); // 近距離のしきい値の円孤
		cvCircle(baseImage, cvPoint(cvRound(ThresM * Scale * sin(angle)) + OffsetX_, cvRound(ThresM * Scale * cos(angle)) + OffsetY_), 1, CV_RGB(32, 32, 32), -1, 8); // 中距離のしきい値の円弧
		cvCircle(baseImage, cvPoint(cvRound(ThresL * Scale * sin(angle)) + OffsetX_, cvRound(ThresL * Scale * cos(angle)) + OffsetY_), 1, CV_RGB(64, 64, 64), -1, 8); // 遠距離のしきい値の円弧
	}

	//**************************************************************************************************
	// OpenCVの準備
	//**************************************************************************************************
	IplImage *dispImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3);	   // 結果表示用画像
	IplImage *distImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 1);	   // センサで取得した距離データを描画する画像
	IplImage *distImageNot = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 1); // センサで取得した距離データを描画した画像を反転
	IplImage *transImage = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_32F, 1);  // 距離画像に変換

	IplImage *dispImage2 = cvCreateImage(cvSize(windowSize.x, windowSize.y), IPL_DEPTH_8U, 3);	   // 結果表示用画像(2つ目のセンサのみ)


	// ウインドウの準備
	cvNamedWindow("Display Image", CV_WINDOW_AUTOSIZE);

	// マウスコールバック関数の登録
	cvSetMouseCallback("Display Image", mouseCallback);

	// 文字フォントの設定
	CvFont myfont;
	cvInitFont(&myfont, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7); // 大きい文字CV_FONT_HERSHEY_COMPLEX,小さい文字CV_FONT_HERSHEY_COMPLEX_SMALL

	//**************************************************************************************************
	// 楕円追跡器(パーティクルフィルタ=Condensation)の準備
	//**************************************************************************************************
	// 追跡器構造体ConDensの作成
	CvConDensation *ConDens = myCreateConDensation(3, 300); // 引数(状態変数ベクトルの次元, サンプル数)
	// サンプルのパラメータを設定する, initValue - 初期値, initMean - 平均, initDeviation - 標準偏差
	CvMat *initValue = cvCreateMat(3, 1, CV_32FC1);
	CvMat *initMean = cvCreateMat(3, 1, CV_32FC1);
	CvMat *initDeviation = cvCreateMat(3, 1, CV_32FC1);
	initValue->data.fl[0] = 0.0;
	initValue->data.fl[1] = 0.0;
	initValue->data.fl[2] = 1080.0; // 初期値(X座標，Y座標，角度) 単位は画素
	initMean->data.fl[0] = 0.0;
	initMean->data.fl[1] = 0.0;
	initMean->data.fl[2] = 0.0; // 平均　(X座標，Y座標，角度) 単位は画素
	initDeviation->data.fl[0] = 5.0;
	initDeviation->data.fl[1] = 5.0;
	initDeviation->data.fl[2] = 20.0; // 分散　(X座標，Y座標，角度) 単位は画素
	// 追跡器構造体Condensの各サンプルの初期化
	myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
	// 追跡器構造体Condensのサンプルの更新
	myConDensUpdateSample(ConDens);
	// センサの位置をセット
	SetSensorPosition_2LS(0, cvPoint(OffsetX[0], OffsetY[0]));
	SetSensorPosition_2LS(1, cvPoint(OffsetX[1], OffsetY[1]));
	SetSensorVisibleRange_2LS(0, sensorVisibleRange[0]);
	SetSensorVisibleRange_2LS(1, sensorVisibleRange[1]);
	// 楕円の輪郭評価点テーブルの作成（長軸半径，短軸半径，刻み角度θ）
	//StoreBodyContourPosition_2LS(18, 9, 10); // 肩の大きさ (横幅　縦幅　角度の刻み) 単位は画素  //元(24, 12, 10)
	StoreBodyContourPosition_2LS(18, 11, 10); // 肩の大きさ (横幅　縦幅　角度の刻み) 単位は画素  //元(24, 12, 10)
	// 円の輪郭評価点テーブルの作成（半径，刻み角度θ）：表示用
	StoreHeadContourPosition_2LS(9, 10); // 頭の大きさ (幅　角度の刻み) 単位は画素  //元(12, 10)
	int prevusrU = -1;	  // 人物位置のx座標
	int prevusrV = -1;	  // 人物位置のy座標
	int prevusrAngl = -1; // 人物胴体の角度

	//**************************************************************************************************
	// Main Loop
	//**************************************************************************************************
	ros::Rate loop_rate(60);
	while (ros::ok())
	{
		// ベース画像で初期化
		cvCopy(baseImage, dispImage);
		cvCopy(baseImage, distanceImage);
		cvZero(intensityImage);
		cvZero(maskImage);
		cvZero(maskImageBinInv);
		cvCopy(baseImage, dispImage2);
		cvCopy(baseImage, distanceImage2);
		cvZero(intensityImage2);
		
		//センサ情報を描画
		int loopNumber = 1;
		for(const auto &itr : handlerList){
			for (int i = 0; i < itr.drawPosition.size(); i++)
			{
				if (itr.urgDistance[i] > ThresS && itr.urgDistance[i] < ThresL && loopNumber == 1)
				{
					cvCircle(distanceImage, itr.drawPosition[i], 1, CV_RGB(0, 255, 255), -1, 8);
					cvCircle(intensityImage, itr.drawPosition[i], 1, CV_RGB(itr.urgIntensity[i], itr.urgIntensity[i], itr.urgIntensity[i]), -1, 8);
					int color1 = itr.urgIntensity[i] * 4 + 64;
					cvCircle(dispImage, itr.drawPosition[i], 1, CV_RGB(64, color1, color1), -1, 8);
				}
				if (itr.urgDistance[i] > ThresS && itr.urgDistance[i] < ThresL && loopNumber == 2)
				{
					cvCircle(distanceImage2, itr.drawPosition[i], 1, CV_RGB(0, 255, 255), -1, 8);
					cvCircle(intensityImage2, itr.drawPosition[i], 1, CV_RGB(itr.urgIntensity[i], itr.urgIntensity[i], itr.urgIntensity[i]), -1, 8);
					int color1 = itr.urgIntensity[i] * 4 + 64;
					cvCircle(dispImage2, itr.drawPosition[i], 1, CV_RGB(64, color1, color1), -1, 8);
				}
				//1センサの観測範囲の境界線
				/*
				if( loopNumber == 1 && (i == 0 || i == (itr.drawPosition.size() - 1)) )
				{
					cv::Point LiderOffset(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
					cvLine(dispImage, LiderOffset, itr.drawPosition[i], CV_RGB(64, 64, 64), 1, 8, 0);
				}
				*/
				if( loopNumber == 1 && (i == 0 || i == (itr.drawPosition.size() - 1)) )
				{
					// // マスク画像作成
					// CvPoint pPolyPoints1[4];
					// CvPoint pPolyPoints2[4];
					// if(i == 0 && itr.offsetY > 0)
					// {
					// 	double kouten_x1 = -itr.drawPosition[i].y * ( ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) / ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) ) + itr.drawPosition[i].x; // 観測点の延長線と画面端の交点

					// 	pPolyPoints1[0] = cvPoint(0, windowSize.y);
					// 	pPolyPoints1[1] = cvPoint(0, windowSize.y * 0.5 - itr.offsetX * Scale);
					// 	pPolyPoints1[2] = cvPoint(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
					// 	pPolyPoints1[3] = cvPoint(kouten_x1, windowSize.y); //Lider -> scan[0] -> 画面端
					// 	cvFillConvexPoly(maskImage, pPolyPoints1, 4, CvScalar(255), 8, 0);
					// 	printf("scan[0]=(%lf,%lf)", itr.drawPosition[i].x, itr.drawPosition[i].y);
					// 	printf("kou1=%lf\n", kouten_x1);
					// }
					// if(i == (itr.drawPosition.size() - 1) && itr.offsetY > 0)
					// {
					// 	double kouten_x2 = -itr.drawPosition[i].y * ( ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) / ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) ) + itr.drawPosition[i].x; // 観測点の延長線と画面端の交点

					// 	pPolyPoints2[0] = cvPoint(0,0);
					// 	pPolyPoints2[1] = cvPoint(0,windowSize.y * 0.5 - itr.offsetX * Scale);
					// 	pPolyPoints2[2] = cvPoint(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
					// 	pPolyPoints2[3] = cvPoint(kouten_x2,0); //Lider -> scan[539] -> 画面端
					// 	cvFillConvexPoly(maskImage, pPolyPoints2, 4, CvScalar(255), 8, 0);
					// 	printf("scan[%d]=(%lf,%lf)", i, itr.drawPosition[i].x, itr.drawPosition[i].y);
					// 	printf("kou2=%lf\n", kouten_x2);
					// }
					// printf("wsx=%d, wsy=%d \n", windowSize.x, windowSize.y);
					// printf("(x,y)=(%lf,%lf) \n", windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
					// cvThreshold(maskImage, maskImageBinInv, 127, 255, CV_THRESH_BINARY_INV);


					// マスク画像作成
					// 右側LiDARの裏側（視野範囲外）領域
					CvPoint pPolyPoints1[4];
					CvPoint pPolyPoints2[4];
					if(i == 0 && itr.offsetY > 0) //右側LiDAR&&1つ目の点
					{
						double kouten_x1 = -itr.drawPosition[i].y * ( ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) / ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) ) + itr.drawPosition[i].x; // 観測点の延長線と画面端(y=0)の交点
						double kouten_y1 = ( ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) / ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) ) * (windowSize.x - itr.drawPosition[i].x) + itr.drawPosition[i].y; // 観測点の延長線と画面端(x=windowSize.x)の交点

						pPolyPoints1[0] = cvPoint(0,0);
						pPolyPoints1[1] = cvPoint(0,windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints1[2] = cvPoint(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints1[3] = cvPoint(kouten_x1,0); //Lider -> scan[0] -> 画面端
						cvFillConvexPoly(maskImage, pPolyPoints1, 4, CvScalar(255), 8, 0);
						//printf("scan[0]=(%lf,%lf)", itr.drawPosition[i].x, itr.drawPosition[i].y);
						//printf("kou1=%lf\n", kouten_x1);
					}
					if(i == (itr.drawPosition.size() - 1) && itr.offsetY > 0) //右側LiDAR&&最後の点
					{
						double kouten_x2 = ( ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) / ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) ) * (windowSize.y - itr.drawPosition[i].y) + itr.drawPosition[i].x; // 観測点の延長線と画面端(y=windowSize.y)の交点
						double kouten_y2 = ( ( itr.drawPosition[i].y - (windowSize.y * 0.5 - itr.offsetX * Scale) ) / ( itr.drawPosition[i].x - (windowSize.x * 0.5 - itr.offsetY * Scale) ) ) * (windowSize.x - itr.drawPosition[i].x) + itr.drawPosition[i].y; // 観測点の延長線と画面端(y=windowSize.x)の交点

						pPolyPoints2[0] = cvPoint(0, windowSize.y);
						pPolyPoints2[1] = cvPoint(0, windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints2[2] = cvPoint(windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
						pPolyPoints2[3] = cvPoint(kouten_x2, windowSize.y); //Lider -> scan[max] -> 画面端
						cvFillConvexPoly(maskImage, pPolyPoints2, 4, CvScalar(255), 8, 0);
						//printf("scan[%d]=(%lf,%lf)", i, itr.drawPosition[i].x, itr.drawPosition[i].y);
						//printf("kou2=%lf\n", kouten_x2);
					}
					//printf("wsx=%d, wsy=%d \n", windowSize.x, windowSize.y);
					//printf("(x,y)=(%lf,%lf) \n", windowSize.x * 0.5 - itr.offsetY * Scale, windowSize.y * 0.5 - itr.offsetX * Scale);
					cvThreshold(maskImage, maskImageBinInv, 127, 255, CV_THRESH_BINARY_INV);
				}
			}
			loopNumber++;
		}

		cvAdd(distanceImage, distanceImage2, distanceImage, maskImageBinInv);
		cvAdd(intensityImage, intensityImage2, intensityImage, maskImageBinInv);
		cvAdd(dispImage, dispImage2, dispImage, maskImageBinInv);

		// 結果の表示
		//cvShowImage("Distance Image", distanceImage);
		cvShowImage("Intensity Image", intensityImage);
		//cvShowImage("Mask Image",maskImage);

		/***************************************************************************************************************/

		cvCvtColor(distanceImage, distImage, CV_BGR2GRAY);
		cvThreshold(distImage, distImageNot, 64.0, 255.0, CV_THRESH_BINARY_INV); // 反射強度が高いものだけ2値化して反転描画(閾値153くらい？)
		cvDistTransform(distImageNot, transImage);								 // 距離画像変換
		//cvNormalize(transImage, distImage, 0.0, 255.0, CV_MINMAX, NULL); // 表示確認用

		// サンプルの更新
		myConDensUpdateSample(ConDens);

		// 各サンプルの尤度を計算
		for (int i = 0; i < 300; ++i)
		{
			ConDens->flConfidence[i] = CalculateBodyLikelihood_2LS(transImage, cvPoint(cvRound(ConDens->flSamples[i][0]), cvRound(ConDens->flSamples[i][1])), cvRound(ConDens->flSamples[i][2]));
		}

		// 先ほど計算した尤度に基づいて推定した状態期待値を計算
		myConDensUpdateByTime(ConDens);

		// 追跡結果(人物位置)を他で使うために取り出す
		int usrU = cvRound(ConDens->State[0]);	  // 人物位置のx座標
		int usrV = cvRound(ConDens->State[1]);	  // 人物位置のy座標
		int usrAngl = cvRound(ConDens->State[2]); // 人物胴体の角度
		usrAngl = (usrAngl % 360 + 360) % 360;    // usrAngl を 0~360 の範囲に制限


		//追跡器の周りの平均反射強度を計算
		//int usrU2 = usrU-30; int usrV2 = usrV-30;
		//if(usrU2<0) usrU2=0; if(usrU2>639-60) usrU2=639-60; if(usrV2<0) usrV2=0; if(usrV2>639-60) usrV2=639-60;
		cvCvtColor(intensityImage, distImage, CV_BGR2GRAY);
		cvSetImageROI(distImage, cvRect(usrU - 30, usrV - 30, 60, 60));
		aveIntensity = (int)(cvSum(distImage).val[0] / cvCountNonZero(distImage));
		cvResetImageROI(distImage);

		//追跡器の周りの平均反射強度が低いときには追跡をやめる
		if (isTracked == 1 && aveIntensity < StopTrackingIntensity)
		{
			initValue->data.fl[0] = NULL;
			initValue->data.fl[1] = NULL;										 // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation); // 追跡器構造体Condensの各サンプルの初期化
			myConDensUpdateSample(ConDens);										 // 追跡器構造体Condensのサンプルの更新
			myConDensUpdateByTime(ConDens);										 // 状態期待値も計算しておく
			isTracked = 0;
		}

		//追跡していないときに，近くで高反射強度が検出された場合，初期値変更して再追跡
		if (isTracked == 0)
		{ //対象損失時
			for(const auto &itr : handlerList){
				int range_of_scanStart = 0;
				int range_of_scanEnd = 0;
				for (int i = 0; i < itr.urgDistance.size(); i++)
				{
					if (itr.urgDistance[i] < StartTrackingDistanceMax && itr.urgIntensity[i] > StartTrackingIntensityMin)
					{																						   //1m以内で正規化反射強度が120以上なら追跡初期化
						initValue->data.fl[0] = cvRound((itr.urgDistance[i] * Scale * itr.sinVal[i]) + OffsetX_ - 20); // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
						initValue->data.fl[1] = cvRound((itr.urgDistance[i] * Scale * itr.cosVal[i]) + OffsetY_);
						initValue->data.fl[2] = prevusrAngl; //昔の向きを引き継ぐ

						myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation); // 追跡器構造体Condensの各サンプルの初期化
						myConDensUpdateSample(ConDens);										 // 追跡器構造体Condensのサンプルの更新
						myConDensUpdateByTime(ConDens);										 // 状態期待値も計算しておく
						isTracked = 1;
						break;
					}
				}
				if(isTracked){
					break;
				}
			}
		}

		//マウスでクリックされたらその位置を初期位置として追跡を開始
		if (lbPressed)
		{
			lbPressed = false; // マウスクリックのフラグを戻す
			initValue->data.fl[0] = (float)lbX;
			initValue->data.fl[1] = (float)lbY;									 // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation); // 追跡器構造体Condensの各サンプルの初期化
			myConDensUpdateSample(ConDens);										 // 追跡器構造体Condensのサンプルの更新
			myConDensUpdateByTime(ConDens);										 // 状態期待値も計算しておく
			usrU = lbX;
			usrV = lbY;
			isTracked = 1;
		}

		// 追跡結果の描画
		if (isTracked == 1)
		{
			DrawBodyContour_2LS(dispImage, cvPoint(usrU, usrV), usrAngl);
			cvRectangle(dispImage, cvPoint(usrU - 30, usrV - 30), cvPoint(usrU + 30, usrV + 30), CV_RGB(255, 255, 255), 1, 8, 0);
			if (aveIntensity < 0)
				aveIntensity = 0;
			char mytext[12];
			sprintf(mytext, "%4d", aveIntensity);
			cvPutText(dispImage, mytext, cvPoint(usrU + 30, usrV + 30), &myfont, CV_RGB(255, 255, 255));
		
		// 角度の描画
		char angleText[12];
		sprintf(angleText, "Ang: %3d", usrAngl);
		cvPutText(dispImage, angleText, cvPoint(usrU + 30, usrV + 50), &myfont, CV_RGB(255, 255, 255));
			}

		// For Follow Tracking
		if (isTracked == 1 && (usrU != 0 && usrV != 0))
		{
			
			double rotX = (usrU - OffsetX_) * cos(OutputTopicOffsetPoseT) - (usrV - OffsetY_) * sin(OutputTopicOffsetPoseT);
			double rotY = (usrU - OffsetX_) * sin(OutputTopicOffsetPoseT) + (usrV - OffsetY_) * cos(OutputTopicOffsetPoseT);
			msgPosePersonFollowing.x = OutputTopicOffsetPoseX + (-1.0) * rotX / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			msgPosePersonFollowing.y = OutputTopicOffsetPoseY - (-1.0) * rotY / (ScanMessageHandler::SCAN_DISTANCE_MAGNIFICATION * Scale);
			//poseTrackingの結果は下向き0=360度，上向き180度,時計回り． これを 上向きpi/2,反時計回り に変換したい
			//msgPosePersonFollowing.theta = OutputTopicOffsetPoseT  + (usrAngl * M_PI * 0.0027778); //これなに？→0.0027778
			msgPosePersonFollowing.theta = OutputTopicOffsetPoseT  + (-usrAngl*M_PI/180.0 + M_PI*1.5);
			pubPosePersonFollowing.publish(msgPosePersonFollowing);

			ROS_WARN("angle= %3d", usrAngl);
			ROS_INFO("msg=%3d", msgPosePersonFollowing.theta);
		}

		if(isTracked == 1){
			prevusrU=usrU;
			prevusrV=usrV;
			prevusrAngl=usrAngl;
	}
		/***************************************************************************************************************/

		// 結果の表示
		cvShowImage("Display Image", dispImage);

		if (cvWaitKey(1) == 'q')
		{
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	// 画像メモリを開放する
	cvReleaseImage(&baseImage);
	cvReleaseImage(&distanceImage);
	cvReleaseImage(&intensityImage);
	cvReleaseImage(&dispImage);
	cvReleaseImage(&distImage);
	cvReleaseImage(&distImageNot);
	cvReleaseImage(&transImage);
	cvReleaseImage(&maskImage);
	cvReleaseImage(&maskImageBin);
	cvReleaseImage(&maskImageBinInv);
	cvReleaseImage(&distanceImage2);
	cvReleaseImage(&intensityImage2);
	cvReleaseImage(&dispImage2);

	// ウインドウを閉じる
	cvDestroyAllWindows();

	return 0;
}
