/*
*  EllipseNormalEvaluation.h
*  必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib
*/

#ifndef __ELLIPSENORMALEVALUATION_H__
#define __ELLIPSENORMALEVALUATION_H__

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

void  SetSensorPosition(CvPoint sensorPos);
void  StoreBodyContourPosition(int majorAxis, int minorAxis, int stepTheta);
void  StoreHeadContourPosition(int headAxis, int stepTheta);

float CalculateBodyLikelihood(IplImage *urgDstImg, CvPoint center, int rotation);
void  DrawBodyContour(IplImage *image, CvPoint center, int rotation);

#endif //__ELLIPSENORMALEVALUATION_H__



