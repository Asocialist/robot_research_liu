
//必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib

#include <windows.h>
#include <process.h>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "aurglib\aurglib.h"

#include "MyCondensation.h"
#include "MyEllipseNormalEvaluation.h"

#define M_PI 3.14159265
int URG_Distance1[1080],URG_Distance2[1080];
int URG_Intensity1[1080],URG_Intensity2[1080];

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

// 追跡中フラグ
int isTracked = 0;
int aveIntensity = 0;

int main(void)
{
	IplImage *baseImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // 分かりやすくするためにセンサ位置や観測範囲などを描画しておくベース画像
	IplImage *distanceImage     = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // センサで取得した距離データを描画する画像
	IplImage *intensityImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // 反射強度用画像

	// ウインドウの準備
	cvNamedWindow("Distance Image", CV_WINDOW_AUTOSIZE );
	cvNamedWindow("Intensity Image", CV_WINDOW_AUTOSIZE );

	// urg1 の開始
	aurglib urg1;
	//if(!urg1.start("COM7",qrk::Lidar::Serial,qrk::Lidar::Distance_intensity)){
	//	printf("Error on Urg_driver::open1\n");	exit(-1);
	//}
	// urg2 の開始
	aurglib urg2;
	if(!urg2.start("COM8",qrk::Lidar::Serial,qrk::Lidar::Distance_intensity)){
		printf("Error on Urg_driver::open2\n");	exit(-1);
	}

	// 距離データ表示用設定
	const int OffsetX = 320; // 画像中でのセンサ位置のX軸方向オフセット
	const int OffsetY = 320; // 画像中でのセンサ位置のY軸方向オフセット
	float Scale  = 0.1f;     // 距離のスケーリング（0.1のとき1cmが1ピクセルに対応）
	float Rotate1 = 180-45;  // センサの回転角度
	float Rotate2 = -45;     // センサの回転角度
	float ThresL = 3000;     // 遠距離のしきい値(3000=3m)
	float ThresM = 1000;     // 中距離のしきい値(1000=1m)
	float ThresS = 200;      // 近距離のしきい値(200=20cm)

	// sin cosのテーブルを作成
	double cosVal1[1080];	double sinVal1[1080];
	for(int ii=0;ii<1080;ii++){
		cosVal1[ii] = cos((ii*0.25+Rotate1)*(M_PI/180));
		sinVal1[ii] = sin((ii*0.25+Rotate1)*(M_PI/180));
	}
	double cosVal2[1080];	double sinVal2[1080];
	for(int ii=0;ii<1080;ii++){
		cosVal2[ii] = cos((ii*0.25+Rotate2)*(M_PI/180));
		sinVal2[ii] = sin((ii*0.25+Rotate2)*(M_PI/180));
	}

	// センサ位置と観測範囲の描画
	cvSetZero(baseImage);
	cvCircle( baseImage, cvPoint(cvRound(OffsetX), cvRound(OffsetY)), 4, CV_RGB(64,64,64), -1, 8); // センサの位置
	for(int jj=0;jj<1080;jj++){
		cvCircle( baseImage, cvPoint(cvRound(ThresS*Scale*sinVal1[jj])+OffsetX, cvRound(ThresS*Scale*cosVal1[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 近距離のしきい値の円孤
		cvCircle( baseImage, cvPoint(cvRound(ThresM*Scale*sinVal1[jj])+OffsetX, cvRound(ThresM*Scale*cosVal1[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 中距離のしきい値の円弧
		cvCircle( baseImage, cvPoint(cvRound(ThresL*Scale*sinVal1[jj])+OffsetX, cvRound(ThresL*Scale*cosVal1[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 遠距離のしきい値の円弧
		cvCircle( baseImage, cvPoint(cvRound(ThresS*Scale*sinVal2[jj])+OffsetX, cvRound(ThresS*Scale*cosVal2[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 近距離のしきい値の円孤
		cvCircle( baseImage, cvPoint(cvRound(ThresM*Scale*sinVal2[jj])+OffsetX, cvRound(ThresM*Scale*cosVal2[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 中距離のしきい値の円弧
		cvCircle( baseImage, cvPoint(cvRound(ThresL*Scale*sinVal2[jj])+OffsetX, cvRound(ThresL*Scale*cosVal2[jj])+OffsetY), 1, CV_RGB(64,64,64), -1, 8); // 遠距離のしきい値の円弧
	}
	
	/***************************************************************************************************************/

	//**************************************************************************************************
    // OpenCVの準備
    //**************************************************************************************************
	IplImage *dispImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 3); // 結果表示用画像
	IplImage *distImage    = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 1); // センサで取得した距離データを描画する画像
	IplImage *distImageNot = cvCreateImage(cvSize(640,640), IPL_DEPTH_8U, 1); // センサで取得した距離データを描画した画像を反転
	IplImage *transImage   = cvCreateImage(cvSize(640,640), IPL_DEPTH_32F,1); // 距離画像に変換

	// ウインドウの準備
	cvNamedWindow("Display Image", CV_WINDOW_AUTOSIZE );

	// マウスコールバック関数の登録
	cvSetMouseCallback("Display Image", mouseCallback);

	// 文字フォントの設定
	CvFont myfont;
	cvInitFont( &myfont, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7 ); // 大きい文字CV_FONT_HERSHEY_COMPLEX,小さい文字CV_FONT_HERSHEY_COMPLEX_SMALL

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

	/***************************************************************************************************************/

	//**************************************************************************************************
	// Main Loop
	//**************************************************************************************************
	for(;;){
		// ベース画像で初期化
		cvCopy(baseImage, dispImage);
		cvCopy(baseImage, distanceImage);
		cvZero(intensityImage);

		// センサで取得した距離と反射強度データを取得
		urg1.getDistIntensity(URG_Distance1, URG_Intensity1);
		urg2.getDistIntensity(URG_Distance2, URG_Intensity2);
		
		// 反射強度正規化用一時変数
		int x[1080],y[1080];
		double URG_Distance_I[1080], URG_Intensity_I[1080];
		double angle[1080], angle_I[1080], a_1[1080], a_2[1080], b_1[1080], b_2[1080], c[1080];
		// 1個目のレーザセンサの反射強度正規化
		for(int i=0;i<1080;i++){
			x[i] = cvRound(URG_Distance1[i]*Scale*sinVal1[i])+OffsetX;
			y[i] = cvRound(URG_Distance1[i]*Scale*cosVal1[i])+OffsetY;
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
			URG_Distance_I[i] = pow((double)URG_Distance1[i],(double)0.287); //正規化のため(距離) r^0.287
			angle_I[i] = pow((double)angle[i],(double)0.196); //正規化のため(入射角) cos^0.196(x)
			URG_Intensity_I[i] = URG_Intensity1[i] * URG_Distance_I[i] / angle_I[i];//反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			URG_Intensity1[i] = cvRound(URG_Intensity_I[i]/200000*255);	if(URG_Intensity1[i]>255) URG_Intensity1[i]=255; if(URG_Intensity1[i]<0) URG_Intensity1[i]=0;
			//正規化(end)
		}
		// 2個目のレーザセンサの反射強度正規化
		for(int i=0;i<1080;i++){
			x[i] = cvRound(URG_Distance2[i]*Scale*sinVal2[i])+OffsetX;
			y[i] = cvRound(URG_Distance2[i]*Scale*cosVal2[i])+OffsetY;
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
			URG_Distance_I[i] = pow((double)URG_Distance2[i],(double)0.287); //正規化のため(距離) r^0.287
			angle_I[i] = pow((double)angle[i],(double)0.196); //正規化のため(入射角) cos^0.196(x)
			URG_Intensity_I[i] = URG_Intensity2[i] * URG_Distance_I[i] / angle_I[i];//反射強度の正規化 正規化値=計測値*r^0.287/cos^0.196(x) (r:距離,cosx:表面法線に対する入射角)
			URG_Intensity2[i] = cvRound(URG_Intensity_I[i]/200000*255);	if(URG_Intensity2[i]>255) URG_Intensity2[i]=255; if(URG_Intensity2[i]<0) URG_Intensity2[i]=0;
			//正規化(end)
		}

		//1個目のセンサ情報を描画
		for(int i=0;i<1080;i++){
			//printf("%f\n",URG_Intensity_I[540]);再帰性反射材時約200000前後
			if(URG_Distance1[i]>ThresS && URG_Distance1[i]<ThresL){
				cvCircle( distanceImage, cvPoint(cvRound(URG_Distance1[i]*Scale*sinVal1[i])+OffsetX-20, cvRound(URG_Distance1[i]*Scale*cosVal1[i])+OffsetY), 1, CV_RGB(0,255,255), -1, 8);
				cvCircle( intensityImage, cvPoint(cvRound(URG_Distance1[i]*Scale*sinVal1[i])+OffsetX-20, cvRound(URG_Distance1[i]*Scale*cosVal1[i])+OffsetY), 1, CV_RGB(URG_Intensity1[i],URG_Intensity1[i],URG_Intensity1[i]), -1, 8);
				int color1 = URG_Intensity1[i]*4+64;
				cvCircle( dispImage, cvPoint(cvRound(URG_Distance1[i]*Scale*sinVal1[i])+OffsetX-20, cvRound(URG_Distance1[i]*Scale*cosVal1[i])+OffsetY), 1, CV_RGB(64,color1,color1), -1, 8);

			}
		}
		//2個目のセンサ情報を描画
		for(int i=0;i<1080;i++){
			//printf("%f\n",URG_Intensity_I[540]);再帰性反射材時約200000前後
			if(URG_Distance2[i]>ThresS && URG_Distance2[i]<ThresL){
				cvCircle( distanceImage, cvPoint(cvRound(URG_Distance2[i]*Scale*sinVal2[i])+OffsetX+19, cvRound(URG_Distance2[i]*Scale*cosVal2[i])+OffsetY-1), 1, CV_RGB(255,255,0), -1, 8);
				cvCircle( intensityImage, cvPoint(cvRound(URG_Distance2[i]*Scale*sinVal2[i])+OffsetX+19, cvRound(URG_Distance2[i]*Scale*cosVal2[i])+OffsetY-1), 1, CV_RGB(URG_Intensity2[i],URG_Intensity2[i],URG_Intensity2[i]), -1, 8);
				int color2 = URG_Intensity2[i]*4+64;
				cvCircle( dispImage, cvPoint(cvRound(URG_Distance2[i]*Scale*sinVal2[i])+OffsetX+19, cvRound(URG_Distance2[i]*Scale*cosVal2[i])+OffsetY-1), 1, CV_RGB(color2,color2,64), -1, 8);
			}
		}
		
		// 結果の表示
		cvShowImage("Distance Image", distanceImage);
		cvShowImage("Intensity Image", intensityImage);

		/***************************************************************************************************************/

		cvCvtColor(distanceImage, distImage, CV_BGR2GRAY);
		cvThreshold(distImage,distImageNot,64.0,255.0,CV_THRESH_BINARY_INV); // 反射強度が高いものだけ2値化して反転描画(閾値153くらい？)
		cvDistTransform(distImageNot, transImage); // 距離画像変換
		//cvNormalize(transImage, distImage, 0.0, 255.0, CV_MINMAX, NULL); // 表示確認用

		// サンプルの更新
		myConDensUpdateSample(ConDens);

		// 各サンプルの尤度を計算
		for(int i=0;i<300;++i){
			ConDens->flConfidence[i] = CalculateBodyLikelihood(transImage, cvPoint(cvRound(ConDens->flSamples[i][0]),cvRound(ConDens->flSamples[i][1])),cvRound(ConDens->flSamples[i][2]));
		}

		// 先ほど計算した尤度に基づいて推定した状態期待値を計算
		myConDensUpdateByTime(ConDens);

		// 追跡結果(人物位置)を他で使うために取り出す
		int usrU    = cvRound(ConDens->State[0]); // 人物位置のx座標
		int usrV    = cvRound(ConDens->State[1]); // 人物位置のy座標
		int usrAngl = cvRound(ConDens->State[2]); // 人物胴体の角度

		//追跡器の周りの平均反射強度を計算
		//int usrU2 = usrU-30; int usrV2 = usrV-30;
		//if(usrU2<0) usrU2=0; if(usrU2>639-60) usrU2=639-60; if(usrV2<0) usrV2=0; if(usrV2>639-60) usrV2=639-60;
		cvCvtColor(intensityImage, distImage, CV_BGR2GRAY);
		cvSetImageROI(distImage, cvRect(usrU-30,usrV-30,60,60));
		aveIntensity = (int)(cvSum(distImage).val[0] / cvCountNonZero(distImage));
		cvResetImageROI(distImage);
		
		//追跡器の周りの平均反射強度が低いときには追跡をやめる
		if(isTracked==1 && aveIntensity<48){
			initValue->data.fl[0] = NULL; initValue->data.fl[1] = NULL; // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);	// 追跡器構造体Condensの各サンプルの初期化
			myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
			myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく			
			isTracked = 0;
		}

		//追跡していないときに，近くで高反射強度が検出された場合，初期値変更して再追跡
		if(isTracked==0){ //対象損失時
			for(int i=0;i<1080;i++){
				if(URG_Distance1[i]<1200 && URG_Intensity1[i]>120){//1m以内で正規化反射強度が120以上なら追跡初期化
					initValue->data.fl[0] = cvRound((URG_Distance1[i]*Scale*sinVal1[i])+OffsetX-20);// サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
					initValue->data.fl[1] = cvRound((URG_Distance1[i]*Scale*cosVal1[i])+OffsetY);
					myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);   // 追跡器構造体Condensの各サンプルの初期化
					myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
					myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく
					isTracked = 1; break;
				}
			}
		}
		if(isTracked==0){ //対象損失時
			for(int i=0;i<1080;i++){
				if(URG_Distance2[i]<1200 && URG_Intensity2[i]>120){//1m以内で正規化反射強度が120以上なら追跡初期化
					initValue->data.fl[0] = cvRound((URG_Distance2[i]*Scale*sinVal2[i])+OffsetX+19);// サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
					initValue->data.fl[1] = cvRound((URG_Distance2[i]*Scale*cosVal2[i])+OffsetY-1);
					myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);   // 追跡器構造体Condensの各サンプルの初期化
					myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
					myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく
					isTracked = 1; break;
				}
			}
		}

		//マウスでクリックされたらその位置を初期位置として追跡を開始
		if(lbPressed){
			lbPressed = false; // マウスクリックのフラグを戻す
			initValue->data.fl[0] = (float)lbX; initValue->data.fl[1] = (float)lbY; // サンプルの初期化パラメータのうち初期値をマウスクリック座標に設定する
			myConDensInitSampleSet(ConDens, initValue, initMean, initDeviation);	// 追跡器構造体Condensの各サンプルの初期化
			myConDensUpdateSample(ConDens); // 追跡器構造体Condensのサンプルの更新
			myConDensUpdateByTime(ConDens); // 状態期待値も計算しておく
			usrU = lbX;	usrV = lbY;
			isTracked = 1;
		}

		// 追跡結果の描画
		if(	isTracked == 1){
			DrawBodyContour(dispImage,cvPoint(usrU,usrV),usrAngl);
			cvRectangle(dispImage,cvPoint(usrU-30,usrV-30),cvPoint(usrU+30,usrV+30),CV_RGB(255,255,255),1,8,0);
			if(aveIntensity<0) aveIntensity=0;
			char mytext[12]; sprintf(mytext,"%4d",aveIntensity);
			cvPutText(dispImage, mytext, cvPoint(usrU+30,usrV+30), &myfont, CV_RGB(255,255,255));
		}

		/***************************************************************************************************************/

		// 結果の表示
		cvShowImage("Display Image", dispImage);
		
		if(cvWaitKey(1) == 'q') {
			break;
		}
	}

	// 画像メモリを開放する
	cvReleaseImage(&baseImage);
	cvReleaseImage(&distanceImage);
	cvReleaseImage(&intensityImage);
	cvReleaseImage(&dispImage);
	cvReleaseImage(&distImage);
	cvReleaseImage(&distImageNot);
	cvReleaseImage(&transImage);

	// ウインドウを閉じる
	cvDestroyAllWindows();

	// urgの終了
	urg1.end();
	urg2.end();
	
    return 1;
}
