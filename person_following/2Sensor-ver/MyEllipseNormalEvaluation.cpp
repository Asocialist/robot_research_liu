/*
*  EllipseNormalEvaluation.cpp 
*  必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib
*/
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "MyEllipseNormalEvaluation.h"

#define IMAGEWIDTH    640	// 画面サイズ幅：VIDEOMODEとあわせる
#define IMAGEHIGHT    640	// 画面サイズ高：VIDEOMODEとあわせる

#define EPS 1.0e-7			//機械誤差

// プロトタイプ宣言
int CalculateBodyContourPosition(CvPoint center, int rotation, int bodyEnableFlg[], CvPoint *bodyPosData);
int CalculateHeadContourPosition(CvPoint center, CvPoint size, int headEnableFlg[], CvPoint *headPosData);

static const int    OutOfImageMargin = 10;                         // 画面端のマージン
static const double CONTOURPI = 3.1415926535897932384626433832795; // 円周率

// ● レーザセンサ画像による胴体評価のための宣言
static CvPoint m_sensorPos;       // センサの位置
// 胴体モデル
static int m_NofBodyEdgePos = 0;  // 胴体輪郭上の点数の保存用
static int m_bodyMajorAxis  = 0;  // 胴体(楕円)の幅
static int m_bodyMinorAxis  = 0;  // 胴体(楕円)の厚み
// 基本楕円の輪郭位置と法線ベクトルのテーブル
CvPoint m_bodyContourPositionTable[36]; // 36個用意(10度刻みまで対応)
CvPoint m_bodyNormalVectorTable[36];    // 36個用意(10度刻みまで対応)
// 頭部モデル
static int m_NofHeadEdgePos = 0;  // 頭部輪郭上の点数の保存用
static int m_headAxis       = 0;  // 頭部(円)の半径
// 基本円の輪郭位置と法線ベクトルのテーブル
CvPoint m_headContourPositionTable[36]; // 36個用意(10度刻みまで対応)
CvPoint m_headNormalVectorTable[36];    // 36個用意(10度刻みまで対応)


//************************************************************
// ● レーザセンサ画像による胴体評価のための関数
//************************************************************
// レーザセンサの位置のセット
void SetSensorPosition(CvPoint sensorPos)
{
	m_sensorPos.x = sensorPos.x;
	m_sensorPos.y = sensorPos.y;
	return;
}

// 胴体の輪郭位置テーブル作成関数（長軸，短軸，刻み角度θ）
void StoreBodyContourPosition(int majorAxis, int minorAxis, int stepTheta)
{
	//IplImage *image  = cvCreateImage(cvSize(IMAGEWIDTH,IMAGEHIGHT), IPL_DEPTH_8U, 1);
	//cvSetZero(image);

	m_bodyMajorAxis = majorAxis;
	m_bodyMinorAxis = minorAxis;

	// 胴体楕円テーブル作成
	for(int i=0; i<360/stepTheta; i++){
		// 胴体輪郭位置
		m_bodyContourPositionTable[i].x = cvRound(majorAxis * cos(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		m_bodyContourPositionTable[i].y = cvRound(minorAxis * sin(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		//printf("position :%d,%d --> ",m_bodyContourPositionTable[i].x,m_bodyContourPositionTable[i].y);
		// 胴体法線ベクトル
		m_bodyNormalVectorTable[i].x = (minorAxis*minorAxis)*m_bodyContourPositionTable[i].x;
		m_bodyNormalVectorTable[i].y = (majorAxis*majorAxis)*m_bodyContourPositionTable[i].y;
		//printf("normalVec:%d,%d\n",m_bodyNormalVectorTable[i].x,m_bodyNormalVectorTable[i].y);
		//cvCircle( image, cvPoint(m_bodyContourPositionTable[i].x+IMAGEWIDTH/2, m_bodyContourPositionTable[i].y+IMAGEHIGHT/2), 2, CV_RGB(255,120,120), 1, 8);
		m_NofBodyEdgePos++;
	}

	//printf("%d\n",m_NofBodyEdgePos);
	//cvShowImage("Sample1", image); cvWaitKey(1000);
	//cvReleaseImage(&image);

	return;
}

// 頭部の輪郭位置テーブル作成関数（半径，刻み角度θ）
void StoreHeadContourPosition(int headAxis, int stepTheta)
{
	//IplImage *image  = cvCreateImage(cvSize(IMAGEWIDTH,IMAGEHIGHT), IPL_DEPTH_8U, 1);
	//cvSetZero(image);

	m_headAxis = headAxis;

	// 頭部円形テーブル作成
	for(int i=0; i<360/stepTheta; i++){
		// 頭部輪郭位置
		m_headContourPositionTable[i].x = cvRound(headAxis * cos(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		m_headContourPositionTable[i].y = cvRound(headAxis * sin(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		//printf("position :%d,%d --> ",m_headContourPositionTable[i].x,m_headContourPositionTable[i].y);
		// 頭部法線ベクトル
		m_headNormalVectorTable[i].x = m_headContourPositionTable[i].x;
		m_headNormalVectorTable[i].y = m_headContourPositionTable[i].y;
		//printf("normalVec:%d,%d\n",m_headNormalVectorTable[i].x,m_headNormalVectorTable[i].y);
		//cvCircle( image, cvPoint(m_headNormalVectorTable[i].x+IMAGEWIDTH/2, m_headNormalVectorTable[i].y+IMAGEHIGHT/2), 2, CV_RGB(255,120,120), 1, 8);
		m_NofHeadEdgePos++;
	}

	//printf("%d\n",m_NofHeadEdgePos);
	//cvShowImage("Sample1", image); cvWaitKey(1000);
	//cvReleaseImage(&image);

	return;
}

// 胴体の輪郭位置計算
int CalculateBodyContourPosition(CvPoint center, int rotation, int bodyEnableFlg[], CvPoint *bodyPosData)
{
	CvPoint bodyContourPos;
	CvPoint bodyContourVec;
	int NofBodyEnablePos = 0;
	// 輪郭位置を計算し，センサ位置との内積からenableFlgを立てる
	for(int i=0; i<m_NofBodyEdgePos; i++){

		// 胴体の回転と並行移動
		bodyContourPos.x = cvRound(cos(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].x - sin(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].y) + center.x;
		bodyContourPos.y = cvRound(sin(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].x + cos(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].y) + center.y;
		bodyContourVec.x = cvRound(cos(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].x - sin(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].y);
		bodyContourVec.y = cvRound(sin(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].x + cos(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].y);

		// 法線ベクトルとの内積をチェック
		int bodyInnerProduct = bodyContourVec.x * (m_sensorPos.x - bodyContourPos.x) + bodyContourVec.y * (m_sensorPos.y - bodyContourPos.y);
		double bodyMag = 
			cvSqrt((double)(m_sensorPos.x-bodyContourPos.x)*(m_sensorPos.x-bodyContourPos.x)+(m_sensorPos.y-bodyContourPos.y)*(m_sensorPos.y-bodyContourPos.y))
			*
			cvSqrt((double)(bodyContourVec.x*bodyContourVec.x)+(bodyContourVec.y*bodyContourVec.y));
		//printf("%4.2f, ",bodyInnerProduct/bodyMag);
		if(bodyInnerProduct/bodyMag > 0.2){
			bodyPosData[i].x = bodyContourPos.x;
			bodyPosData[i].y = bodyContourPos.y;
			bodyEnableFlg[i] = 1;
			NofBodyEnablePos++;
		}else{
			bodyPosData[i].x = bodyContourPos.x;
			bodyPosData[i].y = bodyContourPos.y;
			bodyEnableFlg[i] = 0;
		}
	}

	return NofBodyEnablePos;
}

// 頭部の輪郭位置計算
int CalculateHeadContourPosition(CvPoint center, CvPoint size, int headEnableFlg[], CvPoint *headPosData)
{
	CvPoint headContourPos;
	CvPoint headContourVec;
	int NofHeadEnablePos = 0;
	
	// 輪郭位置を計算し，センサ位置との内積からenableFlgを立てる
	for(int i=0; i<m_NofBodyEdgePos; i++){
		// サイズ変更と並行移動
		headContourPos.x = cvRound( (double)size.x/(double)m_headAxis * (double)m_headContourPositionTable[i].x) + center.x;
		headContourPos.y = cvRound( (double)size.y/(double)m_headAxis * (double)m_headContourPositionTable[i].y) + center.y;
		headContourVec.x = cvRound( (double)size.x/(double)m_headAxis * (double)m_headNormalVectorTable[i].x);
		headContourVec.y = cvRound( (double)size.y/(double)m_headAxis * (double)m_headNormalVectorTable[i].y);

		// 法線ベクトルとの内積をチェック
		int headInnerProduct = headContourVec.x * (m_sensorPos.x - headContourPos.x) + headContourVec.y * (m_sensorPos.y - headContourPos.y);
		double Mag = cvSqrt((double)(m_sensorPos.x-headContourPos.x)*(m_sensorPos.x-headContourPos.x)+(m_sensorPos.y-headContourPos.y)*(m_sensorPos.y-headContourPos.y))*cvSqrt((double)(headContourVec.x*headContourVec.x)+(headContourVec.y*headContourVec.y));
		//printf("%4.2f, ",innerProduct/Mag);
		if(headInnerProduct/Mag > -0.2){
			headPosData[i].x = headContourPos.x;
			headPosData[i].y = headContourPos.y;
			headEnableFlg[i] = 1;
			NofHeadEnablePos++;
		}else{
			headPosData[i].x = headContourPos.x;
			headPosData[i].y = headContourPos.y;
			headEnableFlg[i] = 0;
		}

	}

	return NofHeadEnablePos;
}

// 胴体輪郭の評価関数(輪郭距離画像による評価)
float CalculateBodyLikelihood(IplImage *urgDstImg, CvPoint center, int rotation)
{
	CvPoint edgePositionForCompare; //丸められて同じ位置にきた場合は評価から除くため
	bool    flg  = true;			//同じ位置じゃなかったらtrue
	double  temp = 0;				//尤度格納用
	int		EvalCnt = 0;

	// 楕円の輪郭位置計算
	CvPoint bodyPositionData[36];
	int     bodyEnableFlg[36];
	CalculateBodyContourPosition(center, rotation, bodyEnableFlg, bodyPositionData);

	temp = 0;
	for(int i=0; i<m_NofBodyEdgePos; i++){
		//******* 同じ位置を何度も評価することを防ぐ
		if( i == 0 ){ // 最初の点は無条件で評価するためにフラグを立てる
			edgePositionForCompare.x = bodyPositionData[i].x;
			edgePositionForCompare.y = bodyPositionData[i].y;
			flg = true;
		}else{ // 2点目以降が同じピクセルか？を調べてフラグを立てる
			if(  ( edgePositionForCompare.x == bodyPositionData[i].x ) 
			   &&( edgePositionForCompare.y == bodyPositionData[i].y ) ){
				flg = false;
			}
			else
				flg = true;
		}

		//***** 画面内で同じ位置を見ていないならば、評価する
		if ( flg && (bodyPositionData[i].x>OutOfImageMargin) && (bodyPositionData[i].x<IMAGEWIDTH-OutOfImageMargin) 
			     && (bodyPositionData[i].y>OutOfImageMargin) && (bodyPositionData[i].y<IMAGEHIGHT-OutOfImageMargin) ){
			if(bodyEnableFlg[i]==1){
				CvScalar PixVal = cvGet2D(urgDstImg, bodyPositionData[i].y, bodyPositionData[i].x);
				double tmp = PixVal.val[0];
				if(tmp > temp) temp = tmp;
				EvalCnt++;
			}else{
			}
		}
		//***** 同じ位置を見ているときは評価しない
		else{
		}

		//**** 比較のために今の輪郭の位置を格納
		edgePositionForCompare.x = bodyPositionData[i].x;
		edgePositionForCompare.y = bodyPositionData[i].y;
	}

	//static FILE *fp=fopen("tmp.txt","w");
	//fprintf(fp,"%lf\t%d\n",temp,EvalCnt);
	double ret = exp(-(double)(temp)*(temp)/(5))*exp((double)EvalCnt);
	return temp>EPS?(float)ret:0;
}
   
// レーザセンサ画像への輪郭の描画
void DrawBodyContour(IplImage *image, CvPoint center, int rotation)
{
	// 胴体の輪郭位置の計算と描画
	CvPoint bodyPositionData[36];
	int     bodyEnableFlg[36];
	CalculateBodyContourPosition(center, rotation, bodyEnableFlg, bodyPositionData);

	for(int i=0; i<m_NofBodyEdgePos; i++){
		if(bodyEnableFlg[i]==1){
			cvCircle( image, cvPoint(bodyPositionData[i].x, bodyPositionData[i].y), 2, CV_RGB(255,0,0), 1, 8);
		}else{
			cvCircle( image, cvPoint(bodyPositionData[i].x, bodyPositionData[i].y), 2, CV_RGB(128,128,128), 1, 8);
		}
	}
	
	// 頭部の輪郭位置の計算と描画
	CvPoint headPositionData[36];
	int     headEnableFlg[36];
	CalculateHeadContourPosition(center, cvPoint(m_headAxis,m_headAxis), headEnableFlg, headPositionData);
	
	for(int j=0; j<m_NofHeadEdgePos; j++){
		if(headEnableFlg[j]==1){
			cvCircle( image, cvPoint(headPositionData[j].x, headPositionData[j].y), 2, CV_RGB(255,0,0), 1, 8);
		}else{
			cvCircle( image, cvPoint(headPositionData[j].x, headPositionData[j].y), 2, CV_RGB(128,128,128), 1, 8);
		}
	}

	return;
}

