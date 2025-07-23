/*
*  MyCondensation.h
*  必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib
*/
#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/cxcore.h"

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

typedef struct CvRandState
{
    CvRNG     state;    /* RNG state (the current seed and carry)*/
    int       disttype; /* distribution type */
    CvScalar  param[2]; /* parameters of RNG */
} CvRandState;

CV_EXPORTS void  cvRandInit( CvRandState* state, double param1,
                             double param2, int seed,
                             int disttype CV_DEFAULT(CV_RAND_UNI));

CV_EXPORTS void cvbRand( CvRandState* state, float* dst, int len );

typedef struct CvConDensation{
    int MP; // 観測ベクトルの次元
    int DP; // 累積尤度の保存用
    float* DynamMatr; // 線形ダイナミクスを表す行列
    float* State; // 状態ベクトル
    int SamplesNum; // サンプル数
    float** flSamples; // サンプルベクトルの配列
    float** flNewSamples; // サンプルベクトルのテンポラリ配列
    float* flConfidence; // 各サンプルの確かさ
    float* flCumulative; // 確かさの累積値
    float* Temp; // テンポラリベクトル
    float* RandomSample; // サンプルセットを更新するためのランダムベクトル
    CvRandState* RandS; // ランダムベクトルを生成するための構造体配列
} CvConDensation;

CvConDensation* myCreateConDensation( int DP, int SamplesNum );
void myConDensInitSampleSet( CvConDensation * conDens, CvMat * initvalue, CvMat * mean, CvMat * deviation );
void myConDensUpdateByTime( CvConDensation * ConDens );
void myConDensUpdateSample( CvConDensation * ConDens );
void myConDensUpdateDeviation( CvConDensation * conDens, CvMat * mean, CvMat * deviation );
void myReleaseConDensation( CvConDensation ** ConDensation );
double Dispersion_EffectivenessSample(CvConDensation * ConDens,int x_sum,int y_sum,int DP);

#endif //__MYCONSDENSATION_H__