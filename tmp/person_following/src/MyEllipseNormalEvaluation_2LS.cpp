/*
* MyEllipseNormalEvaluation_2LS.cpp 
* (Updated for OpenCV 4)
*/
#include <stdio.h>
#include <opencv2/opencv.hpp> // 使用现代OpenCV头文件
#include "MyEllipseNormalEvaluation_2LS.h"

// 为了代码简洁，使用cv命名空间
using namespace cv;

// 定义被移除，因为窗口大小由外部传入，不再是硬编码
// #define IMAGEWIDTH    640
// #define IMAGEHIGHT    640

#define EPS 1.0e-7			//机械误差

// 内部函数原型声明 (也使用新的类型)
int CalculateBodyContourPosition_2LS(Point center, int rotation, int bodyEnableFlg[], Point *bodyPosData);
int CalculateHeadContourPosition_2LS(Point center, Point size, int headEnableFlg[], Point *headPosData);

// 全局/静态变量，使用新的类型
static const int    OutOfImageMargin = 10;
static const double CONTOURPI = 3.1415926535897932384626433832795;

static Point m_sensorPos[2];
// 胴体モデル
static int m_NofBodyEdgePos = 0;
static int m_bodyMajorAxis  = 0;
static int m_bodyMinorAxis  = 0;
Point m_bodyContourPositionTable[36];
Point m_bodyNormalVectorTable[36];
// 頭部モデル
static int m_NofHeadEdgePos = 0;
static int m_headAxis       = 0;
Point m_headContourPositionTable[36];
Point m_headNormalVectorTable[36];

Mat m_sensorRange[2];


//************************************************************
// ● レーザセンサ画像による胴体評価のための関数
//************************************************************

// レーザセンサの位置のセット ＠画像座標系
void SetSensorPosition_2LS(int sensorID, Point sensorPos)
{
	if(sensorID < 0 || sensorID >= 2){ // 使用 "||" 更符合逻辑
		fprintf(stderr,"SetSensorPositoin_2LS: irregal sensorID: %d\n", sensorID);
		return;
	}
	m_sensorPos[sensorID] = sensorPos; //可以直接赋值
	return;
}

// レーザセンサの視野範囲のセット 8bit 1ch
void SetSensorVisibleRange_2LS(int sensorID, const Mat& range){
	if(sensorID < 0 || sensorID >= 2){
		fprintf(stderr,"SetSensorVisibleRange: irregal sensorID: %d\n", sensorID);
		return;
	}
	// 不再需要手动管理内存，直接复制数据
    // copyTo 会自动处理内存分配
    range.copyTo(m_sensorRange[sensorID]);
}

// 胴体の輪郭位置テーブル作成関数（長軸，短軸，刻み角度θ）
void StoreBodyContourPosition_2LS(int majorAxis, int minorAxis, int stepTheta)
{
	m_bodyMajorAxis = majorAxis;
	m_bodyMinorAxis = minorAxis;
    m_NofBodyEdgePos = 0; // 每次调用时重置计数

	for(int i=0; i<360/stepTheta; i++){
		// cvRound -> cv::round
		m_bodyContourPositionTable[i].x = round(majorAxis * cos(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		m_bodyContourPositionTable[i].y = round(minorAxis * sin(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		m_bodyNormalVectorTable[i].x = (minorAxis*minorAxis)*m_bodyContourPositionTable[i].x;
		m_bodyNormalVectorTable[i].y = (majorAxis*majorAxis)*m_bodyContourPositionTable[i].y;
		m_NofBodyEdgePos++;
	}
	return;
}

// 頭部の輪郭位置テーブル作成関数（半径，刻み角度θ）
void StoreHeadContourPosition_2LS(int headAxis, int stepTheta)
{
	m_headAxis = headAxis;
    m_NofHeadEdgePos = 0; // 每次调用时重置计数

	for(int i=0; i<360/stepTheta; i++){
		// cvRound -> cv::round
		m_headContourPositionTable[i].x = round(headAxis * cos(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		m_headContourPositionTable[i].y = round(headAxis * sin(i*stepTheta*CONTOURPI/180 + CONTOURPI));
		m_headNormalVectorTable[i].x = m_headContourPositionTable[i].x;
		m_headNormalVectorTable[i].y = m_headContourPositionTable[i].y;
		m_NofHeadEdgePos++;
	}
	return;
}

// 内部函数：胴体の輪郭位置計算
int CalculateBodyContourPosition_2LS(Point center, int rotation, int bodyEnableFlg[], Point *bodyPosData)
{
	Point bodyContourPos;
	Point bodyContourVec;
	int NofBodyEnablePos = 0;

	//どのレーザセンサの視野範囲か判定
	int si;
	for(si=0;si<2;++si){
        // 使用 .at<uchar>(y, x) 安全访问像素
        // 检查 m_sensorRange 是否为空
        if(!m_sensorRange[si].empty() && m_sensorRange[si].at<uchar>(center.y, center.x) > 0){
			break;
		}
	}
	if(si==2){
		// fprintf(stderr,"CalculateBodyContourPosition_2LS: center is not on any sensor's range. Using id#0.\n");
		si = 0; // 默认为0号传感器
	}
	
	for(int i=0; i<m_NofBodyEdgePos; i++){
		// cvRound -> cv::round
		bodyContourPos.x = round(cos(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].x - sin(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].y) + center.x;
		bodyContourPos.y = round(sin(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].x + cos(rotation*CONTOURPI/180)*(double)m_bodyContourPositionTable[i].y) + center.y;
		bodyContourVec.x = round(cos(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].x - sin(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].y);
		bodyContourVec.y = round(sin(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].x + cos(rotation*CONTOURPI/180)*(double)m_bodyNormalVectorTable[i].y);

		int bodyInnerProduct = bodyContourVec.x * (m_sensorPos[si].x - bodyContourPos.x) + bodyContourVec.y * (m_sensorPos[si].y - bodyContourPos.y);
		// cvSqrt -> cv::sqrt
		double bodyMag = 
			sqrt((double)(m_sensorPos[si].x-bodyContourPos.x)*(m_sensorPos[si].x-bodyContourPos.x)+(m_sensorPos[si].y-bodyContourPos.y)*(m_sensorPos[si].y-bodyContourPos.y))
			*
			sqrt((double)(bodyContourVec.x*bodyContourVec.x)+(bodyContourVec.y*bodyContourVec.y));
		
		if(bodyMag > EPS && bodyInnerProduct/bodyMag > 0.2){
			bodyPosData[i] = bodyContourPos;
			bodyEnableFlg[i] = 1;
			NofBodyEnablePos++;
		}else{
			bodyPosData[i] = bodyContourPos;
			bodyEnableFlg[i] = 0;
		}
	}
	return NofBodyEnablePos;
}

// 内部函数：頭部の輪郭位置計算
int CalculateHeadContourPosition_2LS(Point center, Point size, int headEnableFlg[], Point *headPosData)
{
	Point headContourPos;
	Point headContourVec;
	int NofHeadEnablePos = 0;
	
	int si;
	for(si=0;si<2;++si){
        if(!m_sensorRange[si].empty() && m_sensorRange[si].at<uchar>(center.y, center.x) > 0){
			break;
		}
	}
	if(si==2){
		// fprintf(stderr,"CalculateHeadContourPosition_2LS: center not on any range. Using id#0.\n");
		si = 0; // 默认为0号传感器
	}

	for(int i=0; i<m_NofHeadEdgePos; i++){ // 使用头部的点数
		// cvRound -> cv::round
		headContourPos.x = round( (double)size.x/(double)m_headAxis * (double)m_headContourPositionTable[i].x) + center.x;
		headContourPos.y = round( (double)size.y/(double)m_headAxis * (double)m_headContourPositionTable[i].y) + center.y;
		headContourVec.x = round( (double)size.x/(double)m_headAxis * (double)m_headNormalVectorTable[i].x);
		headContourVec.y = round( (double)size.y/(double)m_headAxis * (double)m_headNormalVectorTable[i].y);

		int headInnerProduct = headContourVec.x * (m_sensorPos[si].x - headContourPos.x) + headContourVec.y * (m_sensorPos[si].y - headContourPos.y);
        // cvSqrt -> cv::sqrt
		double Mag = sqrt((double)(m_sensorPos[si].x-headContourPos.x)*(m_sensorPos[si].x-headContourPos.x)+(m_sensorPos[si].y-headContourPos.y)*(m_sensorPos[si].y-headContourPos.y))*sqrt((double)(headContourVec.x*headContourVec.x)+(headContourVec.y*headContourVec.y));

		if(Mag > EPS && headInnerProduct/Mag > -0.2){
			headPosData[i] = headContourPos;
			headEnableFlg[i] = 1;
			NofHeadEnablePos++;
		}else{
			headPosData[i] = headContourPos;
			headEnableFlg[i] = 0;
		}
	}
	return NofHeadEnablePos;
}

// 胴体輪郭の評価関数(輪郭距離画像による評価)
float CalculateBodyLikelihood_2LS(const Mat& urgDstImg, Point center, int rotation)
{
	Point edgePositionForCompare;
	bool    flg  = true;
	double  temp = 0;
	int		EvalCnt = 0;

	Point bodyPositionData[36];
	int   bodyEnableFlg[36];
	CalculateBodyContourPosition_2LS(center, rotation, bodyEnableFlg, bodyPositionData);

	temp = 0;
	for(int i=0; i<m_NofBodyEdgePos; i++){
		if( i == 0 ){
			edgePositionForCompare = bodyPositionData[i];
			flg = true;
		}else{
			flg = (edgePositionForCompare != bodyPositionData[i]);
		}

        // 使用 .cols 和 .rows 访问图像尺寸
		if ( flg && (bodyPositionData[i].x > OutOfImageMargin) && (bodyPositionData[i].x < urgDstImg.cols - OutOfImageMargin) 
			     && (bodyPositionData[i].y > OutOfImageMargin) && (bodyPositionData[i].y < urgDstImg.rows - OutOfImageMargin) ){
			if(bodyEnableFlg[i]==1){
                // 使用 .at<float>() 安全地访问像素，因为距离变换图是 CV_32F
				float tmp = urgDstImg.at<float>(bodyPositionData[i].y, bodyPositionData[i].x);
				if(tmp > temp) temp = tmp;
				EvalCnt++;
			}
		}
		edgePositionForCompare = bodyPositionData[i];
	}

	double ret = exp(-(double)(temp)*(temp)/(5.0)) * exp((double)EvalCnt); // 5.0 避免整数除法
	return temp > EPS ? (float)ret : 0.0f;
}
   
// レーザセンサ画像への輪郭の描画
void DrawBodyContour_2LS(Mat& image, Point center, int rotation)
{
	Point bodyPositionData[36];
	int   bodyEnableFlg[36];
	CalculateBodyContourPosition_2LS(center, rotation, bodyEnableFlg, bodyPositionData);

	for(int i=0; i<m_NofBodyEdgePos; i++){
        // cvCircle -> cv::circle, CV_RGB -> cv::Scalar(BGR)
		if(bodyEnableFlg[i]==1){
			circle( image, bodyPositionData[i], 2, Scalar(0,0,255), 1, 8); // Red
		}else{
			circle( image, bodyPositionData[i], 2, Scalar(128,128,128), 1, 8); // Gray
		}
	}
	
	Point headPositionData[36];
	int   headEnableFlg[36];
	CalculateHeadContourPosition_2LS(center, Point(m_headAxis,m_headAxis), headEnableFlg, headPositionData);
	
	for(int j=0; j<m_NofHeadEdgePos; j++){
		if(headEnableFlg[j]==1){
			circle( image, headPositionData[j], 2, Scalar(0,0,255), 1, 8); // Red
		}else{
			circle( image, headPositionData[j], 2, Scalar(128,128,128), 1, 8); // Gray
		}
	}
	return;
}