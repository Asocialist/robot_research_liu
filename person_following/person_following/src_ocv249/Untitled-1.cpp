#include <sstream>
#include <vector>
#include <algorithm>

#include "ros/ros.h"
#include "json11.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

// ////////////////
// // Defines
// ////////////////


// ////////////////
// // Variables
// ////////////////


// ////////////////
// // Functions
// ////////////////

#include <iostream>
#include <memory>


//必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "MyEllipseNormalEvaluation.h"
#include "MyCondensation.h"

#include "sensor_msgs/LaserScan.h"




#define M_PI 3.14159265

constexpr int LASER_STEP_NUM = 1080;

int URG_Distance[LASER_STEP_NUM];
int URG_Intensity[LASER_STEP_NUM];
double URG_Distance_I[LASER_STEP_NUM],URG_Intensity_I[LASER_STEP_NUM],angle[LASER_STEP_NUM],angle_I[LASER_STEP_NUM],a_1[LASER_STEP_NUM],a_2[LASER_STEP_NUM],b_1[LASER_STEP_NUM],b_2[LASER_STEP_NUM],c[LASER_STEP_NUM];//反射強度正規化用

//車椅子のCOMポート番号(上)
char ComJW[16] = "\\\\.\\COM4";
char ComURG[16] = "COM3";

// マウスイベントの取得
bool lbPressed = false;
int lbX = 0;
int lbY = 0;

// マウスコールバック関数
void mouseCallback(int event, int x, int y, int flag, void *param){
	if(event==CV_EVENT_LBUTTONDOWN){
		lbPressed = true;
		lbX = x;
		lbY = y;
	}
	return;
}

int main(int argc, char **argv)
{
    /*////
	// For Follow Tracking
	std::unique_ptr<FollowTrackingServer> ftServer;
	//FollowTrackingServer* ftServer = nullptr;
	if (argc >= 2) {
		ftServer = std::make_unique<FollowTrackingServer>(argv[1]);
		//ftServer = new FollowTrackingServer(argv[1]);
	}
	else {
		std::cerr << "invalid argument." << std::endl;
		exit(1);
	}
    */////



	IplImage *dispImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // 結果表示用画像
	IplImage *baseImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // 分かりやすくするためにセンサ位置や観測範囲などを描画しておくベース画像
	IplImage *distImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 1); // センサで取得した距離データを描画する画像
	IplImage *distImageNot = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 1); // センサで取得した距離データを描画した画像を反転
	IplImage *transImage   = cvCreateImage(cvSize(640,640), IPL_DEPTH_32F,1); // 距離画像に変換
	IplImage *intensityImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // 反射強度用画像

	// ウインドウの準備
	cvNamedWindow("Display Image", CV_WINDOW_AUTOSIZE );
	cvNamedWindow("Intensity Image", CV_WINDOW_AUTOSIZE );

	// マウスコールバック関数の登録
	cvSetMouseCallback("Display Image",mouseCallback);
	// 文字フォントの設定
	CvFont myfont;
	cvInitFont( &myfont, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, 1 ); // 大きい文字CV_FONT_HERSHEY_COMPLEX,小さい文字CV_FONT_HERSHEY_COMPLEX_SMALL
	// URG の開始
	/*////
	aurglib urg;
	strcpy(ComURG, ftServer->GetURGPort().c_str());
	if(!urg.start(ComURG, qrk::Lidar::Serial,qrk::Lidar::Distance_intensity)){
		fprintf(stderr,"Error on Urg_driver::open\n");
		exit(-1);
	}
    /*////


	// 距離データ表示用設定
	const int OffsetX = 320; // 画像中でのセンサ位置のX軸方向オフセット
	const int OffsetY = 320; // 画像中でのセンサ位置のY軸方向オフセット
	float Scale  = 0.1f;     // 距離のスケーリング（0.1のとき1cmが1ピクセルに対応）
	float Rotate = 45;       // センサの回転角度
	float ThresL = 6000;     // 遠距離のしきい値(3000=3m)
	float ThresM = 500;      // 中距離のしきい値(500=50cm)
	float ThresS = 200;      // 近距離のしきい値(200=20cm)
	int INTENS_S = 120000; //正規化反射強度の下限
	int INTENS_L = 200000; //正規化反射強度の上限
	int usrU=0,usrV=0,usrAngl=0;// 人物位置のx座標,y座標,人物胴体の角度
	// sin cosのテーブルを作成
	double cosVal[LASER_STEP_NUM];	double sinVal[LASER_STEP_NUM];
	for(int ii=0;ii<LASER_STEP_NUM;ii++){
		cosVal[ii] = cos((ii*0.25+Rotate)*(M_PI/180));
		sinVal[ii] = sin((ii*0.25+Rotate)*(M_PI/180));
	}

	// センサ位置と観測範囲の描画
	cvSetZero(baseImage);
	cvCircle( baseImage, cvPoint(cvRound(OffsetX), cvRound(OffsetY)), 4, CV_RGB(64,64,64), -1, 8); // センサの位置
	for(int jj=0;jj<LASER_STEP_NUM;jj++){
		cvCircle( baseImage, cvPoint(cvRound(ThresS*Scale*sinVal[jj])+OffsetX, cvRound(ThresS*Scale*cosVal[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 近距離のしきい値の円孤
		cvCircle( baseImage, cvPoint(cvRound(ThresM*Scale*sinVal[jj])+OffsetX, cvRound(ThresM*Scale*cosVal[jj])+OffsetY), 1, CV_RGB(32,32,32), -1, 8); // 中距離のしきい値の円弧
		cvCircle( baseImage, cvPoint(cvRound(ThresL*Scale*sinVal[jj])+OffsetX, cvRound(ThresL*Scale*cosVal[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 遠距離のしきい値の円弧
	}

	//**************************************************************************************************
    // 楕円追跡器(パーティクルフィルタ=Condensation)の準備
    //**************************************************************************************************
    
	// 追跡器構造体ConDensの作成
	CvConDensation *ConDens = myCreateConDensation(3, 300); // 引数(状態変数ベクトルの次元, サンプル数)
	// サンプルのパラメータを設定する, initValue - 初期値, initMean - 平均, initDeviation - 標準偏差
	CvMat *initValue     = cvCreateMat(3, 1, CV_32FC1);
	CvMat *initMean      = cvCreateMat(3, 1, CV_32FC1);
	CvMat *initDeviation = cvCreateMat(3, 1, CV_32FC1);
	initValue->data.fl[0]    = 0.0; initValue->data.fl[1]     = 0.0; initValue->data.fl[2]     =  0.0; // 初期値(X座標，Y座標，角度) 単位は画素
	initMean->data.fl[0]     = 0.0; initMean->data.fl[1]      = 0.0; initMean->data.fl[2]      =  0.0; // 平均　(X座標，Y座標，角度) 単位は画素
	initDeviation->data.fl[0]= 5.0; initDeviation->data.fl[1] = 5.0; initDeviation->data.fl[2] = 20.0; // 分散　(X座標，Y座標，角度) 単位は画素
	// 追跡器構造体Condensの各サンプルの初期化
	myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);
	// 追跡器構造体Condensのサンプルの更新
	myConDensUpdateSample(ConDens);
	// センサの位置をセット
	SetSensorPosition(cvPoint(OffsetX, OffsetY));
	// 楕円の輪郭評価点テーブルの作成（長軸半径，短軸半径，刻み角度θ）
	StoreBodyContourPosition(24,12,10); // 肩の大きさ (横幅　縦幅　角度の刻み) 単位は画素
	// 円の輪郭評価点テーブルの作成（半径，刻み角度θ）：表示用
	StoreHeadContourPosition(12,10);    // 頭の大きさ (幅　角度の刻み) 単位は画素


	int x[LASER_STEP_NUM],y[LASER_STEP_NUM];
	int k=0;//状態,マウスクリック
	int count_1=0; //有効サンプル数
	int sum[3];//有効サンプル分散算出のための累計格納変数
	double dis[3];//有効サンプルの分散(x,y,角度)

	while(true){
	//for(int r=0;r<100000;r++){//r:サイクル数
		// ベース画像で初期化
		cvCopy(baseImage, dispImage);
		cvZero(distImage);
		cvCopy(baseImage, intensityImage);

		//サイクルごとに有効サンプルの分散を求めるための初期化
		for(int lp=0;lp<3;lp++){//(0:x,1:y,2:角度)
			sum[lp]=0;//累計値初期化
			dis[lp]=0.0;//分散値初期化
		}
		// センサで取得した距離データを描画
        /*////
		urg.getDistIntensity(URG_Distance,URG_Intensity); // 距離データの取得
        /*////
		for(int i = 0; i < LASER_STEP_NUM; i++){
			x[i] = cvRound(URG_Distance[i]*Scale*sinVal[i])+OffsetX;
			y[i] = cvRound(URG_Distance[i]*Scale*cosVal[i])+OffsetY;

			//反射強度正規化(begin) 正規化反射強度:URG_Intensity_I
			if(i!=0){
				a_1[i] = OffsetX - x[i];//ベクトルaのx成分
				a_2[i] = OffsetY - y[i];//ベクトルaのy成分
				b_1[i] = x[i-1] - x[i];//ベクトルbのx成分
				b_2[i] = y[i-1] - y[i];//ベクトルbのy成分
				c[i] = a_1[i] * b_1[i] + a_2[i] * b_2[i] / 
					sqrt((double)(a_1[i] * a_1[i] + a_2[i] * a_2[i])) * sqrt((double)(b_1[i] * b_1[i] + b_2[i] * b_2[i]));//2ベクトルのなす角からcos(90-x)を求める
				angle[i] = sqrt( 1 - (c[i] * c[i]));//表面の法線ベクトルに対する入射角 sin^2 + cos^2 = 1よりsinを求め cosx = sin(90-x)より cosxをもとめた
			}
			URG_Distance_I[i] = pow((double)URG_Distance[i],(double)0.287); //正規化のため(距離) r^0.287
			angle_I[i] = pow((double)angle[i],(double)0.196); //正規化のため(入射角) cos^0.196(x)
			URG_Intensity_I[i] = URG_Intensity[i] * URG_Distance_I[i] / angle_I[i];//反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			//正規化(end)

			//センサからの距離情報を描画
			//printf("%f\n",URG_Intensity_I[540]);再規制反射材時約200000前後
			if(URG_Distance[i]>ThresS && URG_Distance[i]<ThresL){
				cvCircle( dispImage, cvPoint(cvRound(URG_Distance[i]*Scale*sinVal[i])+OffsetX, cvRound(URG_Distance[i]*Scale*cosVal[i])+OffsetY), 1, CV_RGB(255,255,255), -1, 8);
				cvCircle( distImage, cvPoint(cvRound(URG_Distance[i]*Scale*sinVal[i])+OffsetX, cvRound(URG_Distance[i]*Scale*cosVal[i])+OffsetY), 1, CV_RGB(255,255,255), -1, 8);
				if(URG_Intensity_I[i]>INTENS_S && URG_Intensity_I[i]<INTENS_L){
					cvCircle( intensityImage,cvPoint(x[i],y[i]), 1, CV_RGB(255,255,255), -1, 8);
				}
			}
		}
		cvNot(distImage,distImageNot); // 反転
		cvDistTransform(distImageNot, transImage); // 距離画像変換
		// サンプルの更新
		myConDensUpdateSample(ConDens);
		// 各サンプルの尤度を計算
		for(int i=0;i<300;++i){
			ConDens->flConfidence[i] = CalculateBodyLikelihood(transImage, cvPoint(cvRound(ConDens->flSamples[i][0]),cvRound(ConDens->flSamples[i][1])),cvRound(ConDens->flSamples[i][2]));
		}

		//有効サンプル算出
		count_1 = 0; //count_1:更新数(受け継がれるサンプルの数)
		int j_before = -1; //前回の更新のjの値を格納
		if(k==1){ //k=1:マウスクリック後
			 for(int o = 0; o<ConDens->SamplesNum; o++){
				int j = 1;
				while((ConDens->flCumulative[j]<=(float)o*(float)ConDens->MP/300.0+0.000001f)&&(j<(ConDens->SamplesNum-1))){
					j++;
				}
				if(j_before != j){ //更新時
					count_1++;
					for(int lp = 0; lp < 3; lp++){
						sum[lp] +=  ConDens->flSamples[o][lp];//有効サンプルの値を累計(分散を求めるため)0:x,1:y,2:角度
					}
					j_before = j;
				}
			}
		}

		//分散値計算
		for(int lp = 0; lp < 3; lp++){//有効サンプルの分散を計算(lp-次元　0:x,1:y,2:角度)
			dis[lp] =  Dispersion_EffectivenessSample(ConDens,sum[lp],count_1,lp);//Dispersion_EffectivenessSample(ConDens:サンプル情報,sum:有効サンプル値の累計値,count_1:有効サンプル数,lp:次元)
		}

		// 先ほど計算した尤度に基づいて推定した状態期待値を計算
		myConDensUpdateByTime(ConDens);
		 // 追跡結果(人物位置)を他で使うために取り出す
		usrU    = cvRound(ConDens->State[0]); // 人物位置のx座標
		usrV    = cvRound(ConDens->State[1]); // 人物位置のy座標
		usrAngl = cvRound(ConDens->State[2]); // 人物胴体の角度

		// 追跡結果の描画
		DrawBodyContour(dispImage,cvPoint(usrU,usrV),usrAngl);


		//マウスでクリックされたらその位置を初期位置として追跡を開始
		if(lbPressed){
			lbPressed = false; // マウスクリックのフラグを戻す
			initValue->data.fl[0] = (float)lbX; initValue->data.fl[1] = (float)lbY; // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);	// 追跡器構造体Condensの各サンプルの初期化
			myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
			myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく
			k=1;//マウスクリック後
		}


		//出力値の作成
		//vehicle.move(speedY,speedX)
		//speedX:左右方向の旋回速度(-100~100)
		//speedY:前後方向の直進速度(-100~100)
		int orgPosX = 320;//ニュートラル位置X//320
		int orgPosY = 240;//ニュートラル位置Y//240
		int diffX = orgPosX - usrU;//車椅子からみた相対的な対象者のx位置
		int diffY = orgPosY - usrV;//車椅子からみた相対的な対象者のy位置
		//命令の出力
		//if(-120<diffX && diffX<120 && -40<diffY && diffY<120){
		//	vehicle.move((diffY*2),(diffX*2));//*2
		//	Sleep(1);
		//}else{
		//	vehicle.stop();
		//	Sleep(1);
		//}

		//対象損失時 高反射強度が検出された場合、初期値変更して再追跡
		int DiffX,DiffY;//センサ位置からの相対距離
		for(int i=0;i<LASER_STEP_NUM;i++){
		DiffX = OffsetX - x[i];
		DiffY = OffsetY - y[i];
			if(k!=1){ //対象損失時:k=0
				if(-120<DiffX && DiffX<120 && -40<DiffY && DiffY<120){//射程枠内に
					if(URG_Intensity_I[i]>INTENS_S && URG_Intensity_I[i]<INTENS_L){//正規化反射強度が閾値以上なら初期値変更
						initValue->data.fl[0] = (float)x[i]; initValue->data.fl[1] = (float)y[i]; // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
						myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);	// 追跡器構造体Condensの各サンプルの初期化
						myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
						myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく
						k=1;
					}
				}
			}
		}

		//車椅子からみた相対的な対象者のx位置,y位置,状態を描画
		char posText[12];
		sprintf(posText, "%3d,%3d,%3d",diffX,diffY,k);
		cvPutText(dispImage,posText,cvPoint(20,20),&myfont,CV_RGB(255,255,255));

		// For Follow Tracking
		/*////
		if (usrU != 0 && usrV != 0 && k != 0){
			ftServer->SendTracking(diffX, diffY, k);
		}
		/*////
		
		// 動作範囲の描画
		cvRectangle(dispImage, cvPoint(orgPosX-120, orgPosY-120), cvPoint(orgPosX+120, orgPosY+40), CV_RGB(255, 255, 255) );


		//有効サンプルの分散次第でマーキングを外す
		if((dis[0] > 2500.0) || (dis[1] >2500.0)){//有効サンプル分散が閾値を超えるなら
			initValue->data.fl[0] = NULL; initValue->data.fl[1] = NULL; // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);	// 追跡器構造体Condensの各サンプルの初期化
			myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
			myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく
			k=0;
			printf("外す処理\n");
			
		}
		// 結果の表示
		cvShowImage("Display Image", dispImage);
		cvShowImage("Intensity Image", intensityImage);
		if(cvWaitKey(1) == 'q') {
			break; // exit main roop
			//vehicle.stop();
			//Sleep(1);
			// URG の終了
			/*////
			urg.end();
			/*////
		}
	}

	// 追跡器構造体ConDensのメモリを開放する
	myReleaseConDensation(&ConDens);
	// 行列，ベクトルのメモリを開放する
	cvReleaseMat(&initValue);
	cvReleaseMat(&initMean);
	cvReleaseMat(&initDeviation);
	// 画像メモリを開放する
	cvReleaseImage(&dispImage);
	cvReleaseImage(&baseImage);
	cvReleaseImage(&distImage);
	cvReleaseImage(&distImageNot);
	cvReleaseImage(&transImage);
	cvReleaseImage(&intensityImage);
	// ウインドウを閉じる
	cvDestroyWindow("Display Image");
	cvDestroyWindow("Intensity Image");

	//vehicle.stop();
	//Sleep(1);
	// URG の終了
    /*////
	urg.end();
    /*////
    return 1;
}


int main2(int argc, char **argv){

    // ---Configure---
    std::string defNodeName                 = "person_following";

    // // ---Configure---

    // {// ---Load configure---
    //     if (argc >= 2) {
    //         auto configure = json11::LoadJsonFile(argv[1]);
    //         if (configure == nullptr) {
    //             std::cerr << "cannnoot read configure file." << std::endl;
    //         }
    //         else{
    //             defNodeName                     = configure["defNodeName"].is_null() ? defNodeName : configure["defNodeName"].string_value();
    //             defSrvNameAddDestination        = configure["defSrvNameAddDestination"].is_null() ? defSrvNameAddDestination : configure["defSrvNameAddDestination"].string_value();
    //             defSrvNameDeleteDestination     = configure["defSrvNameDeleteDestination"].is_null() ? defSrvNameDeleteDestination : configure["defSrvNameDeleteDestination"].string_value();
    //             defSrvNameShowQueue             = configure["defSrvNameShowQueue"].is_null() ? defSrvNameShowQueue : configure["defSrvNameShowQueue"].string_value();
    //             defTopicNamePose                = configure["defTopicNamePose"].is_null() ? defTopicNamePose : configure["defTopicNamePose"].string_value();
    //             defTopicNameStatus              = configure["defTopicNameStatus"].is_null() ? defTopicNameStatus : configure["defTopicNameStatus"].string_value();
    //             defPathFile                     = configure["defPathFile"].is_null() ? defPathFile : configure["defPathFile"].string_value();
    //             defTopicNameQueueinfo           = configure["defTopicNameQueueinfo"].is_null() ? defTopicNameQueueinfo : configure["defTopicNameQueueinfo"].string_value();
    //             defArriveDecisionDistance       = configure["defArriveDecisionDistance"].is_null() ? defArriveDecisionDistance : configure["defArriveDecisionDistance"].int_value();
    //         }

    //     }
    //     else {
    //         //     std::cerr << "invalid argument." << std::endl;
    //         //     exit(1);
    //     }
    // }// ---Load configure---

    // ---ROS Initialize---
    ros::init(argc, argv, defNodeName.c_str());
    ros::NodeHandle nodeHandle;

    // ros::Subscriber subPose =  nodeHandle.subscribe(defTopicNamePose.c_str(), 100, 
    // +[](const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg) {     
    //     return;
    // });


    // ---Main Loop---
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ---Main Loop---

    return 0;
}

