/*
* EllipseNormalEvaluation.h
* (Updated for OpenCV 4)
*/

#ifndef __ELLIPSENORMALEVALUATION_H__
#define __ELLIPSENORMALEVALUATION_H__

// 使用现代OpenCV头文件
#include <opencv2/opencv.hpp>

// レーザセンサの位置のセット ＠画像座標系
// CvPoint -> cv::Point
void  SetSensorPosition_2LS(int sensorID, cv::Point sensorPos);

// レーザセンサの視野範囲のセット 8bit 1ch
// IplImage* -> const cv::Mat& (作为输入图像，使用const引用)
void SetSensorVisibleRange_2LS(int sensorID, const cv::Mat& range);

// 这两个函数的签名没有OpenCV类型，保持不变
void  StoreBodyContourPosition_2LS(int majorAxis, int minorAxis, int stepTheta);
void  StoreHeadContourPosition_2LS(int headAxis, int stepTheta);

// 胴体の輪郭位置計算
// IplImage* -> const cv::Mat&, CvPoint -> cv::Point
float CalculateBodyLikelihood_2LS(const cv::Mat& urgDstImg, cv::Point center, int rotation);

// 頭部の輪郭位置計算
// IplImage* -> cv::Mat& (需要修改图像，使用非const引用)
void  DrawBodyContour_2LS(cv::Mat& image, cv::Point center, int rotation);

#endif //__ELLIPSENORMALEVALUATION_H__