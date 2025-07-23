/*
*  EllipseNormalEvaluation.h
*  必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib
*/

#ifndef __ELLIPSENORMALEVALUATION_H__
#define __ELLIPSENORMALEVALUATION_H__

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

// レーザセンサの位置のセット ＠画像座標系
void  SetSensorPosition_2LS(int sensorID, CvPoint sensorPos);
// レーザセンサの視野範囲のセット 8bit 1ch
void SetSensorVisibleRange_2LS(int sensorID, IplImage *range);
// 胴体の輪郭位置テーブル作成関数（長軸，短軸，刻み角度θ）
void  StoreBodyContourPosition_2LS(int majorAxis, int minorAxis, int stepTheta);
// 頭部の輪郭位置テーブル作成関数（半径，刻み角度θ）
void  StoreHeadContourPosition_2LS(int headAxis, int stepTheta);

// 胴体の輪郭位置計算
float CalculateBodyLikelihood_2LS(IplImage *urgDstImg, CvPoint center, int rotation);
// 頭部の輪郭位置計算
void  DrawBodyContour_2LS(IplImage *image, CvPoint center, int rotation);

#endif //__ELLIPSENORMALEVALUATION_H__



