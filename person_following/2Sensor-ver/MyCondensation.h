/*
*  MyCondensation.h
*  必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib
*/
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>

#ifndef __MYCONSDENSATION_H__
#define __MYCONSDENSATION_H__

// ベクトル計算用低レベル関数の定義(OpenCV1.0からコピペ)
#define icvCheckVector_32f( ptr, len )
#define icvSetZero_32f( dst, cols, rows ) memset((dst),0,(rows)*(cols)*sizeof(float))
#define icvCopyVector_32f( src, len, dst ) memcpy((dst),(src),(len)*sizeof(float))

CV_INLINE void icvAddVector_32f( const float* src1, const float* src2, float* dst, int len ){
    int i;
    for( i = 0; i < len; i++ )
        dst[i] = src1[i] + src2[i];

    icvCheckVector_32f( dst, len );
}

CV_INLINE void icvScaleVector_32f( const float* src, float* dst, int len, double scale ){
    int i;
    for( i = 0; i < len; i++ )
        dst[i] = (float)(src[i]*scale);

    icvCheckVector_32f( dst, len );
}

CvConDensation* myCreateConDensation( int DP, int SamplesNum );
void myConDensInitSampleSet( CvConDensation * conDens, CvMat * initvalue, CvMat * mean, CvMat * deviation );
void myConDensUpdateByTime( CvConDensation * ConDens );
void myConDensUpdateSample( CvConDensation * ConDens );
void myConDensUpdateDeviation( CvConDensation * conDens, CvMat * mean, CvMat * deviation );
void myReleaseConDensation( CvConDensation ** ConDensation );

#endif //__MYCONSDENSATION_H__