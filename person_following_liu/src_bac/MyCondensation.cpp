/*
*  MyCondensation.cpp
*  必要ライブラリ opencv_core249d.lib opencv_imgproc249d.lib opencv_legacy249d.lib opencv_highgui249d.lib
*/
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include "MyCondensation.h"

// エラー出力ファイル
extern FILE *fp;

//*************************************************************
// Condensation構造体を初期化する関数
//*************************************************************
CvConDensation* myCreateConDensation( int DP, int SamplesNum )
{
    int i;
    CvConDensation *CD = 0;
    
    if( DP < 0 || SamplesNum < 0 ) { printf("Condensation Error.\n"); return NULL; }
    
    /* allocating memory for the structure */
    CD = (CvConDensation *) cvAlloc( sizeof( CvConDensation ));
    /* setting structure params */
    CD->SamplesNum = SamplesNum;
    CD->DP = DP;
	// 使われてないMPを累積尤度の保存に使う
    CD->MP = 0;
    /* allocating memory for structure fields */
    CD->flSamples       = (float **) cvAlloc( sizeof( float * ) * SamplesNum );
    CD->flNewSamples    = (float **) cvAlloc( sizeof( float * ) * SamplesNum );
    CD->flSamples[0]    = (float *)  cvAlloc( sizeof( float ) * SamplesNum * DP );
    CD->flNewSamples[0] = (float *)  cvAlloc( sizeof( float ) * SamplesNum * DP );

    /* setting pointers in pointer's arrays */
    for( i = 1; i < SamplesNum; i++ )
    {
        CD->flSamples[i]    = CD->flSamples[i - 1] + DP;
        CD->flNewSamples[i] = CD->flNewSamples[i - 1] + DP;
    }

    CD->State        = (float *) cvAlloc( sizeof( float ) * DP );
    CD->DynamMatr    = (float *) cvAlloc( sizeof( float ) * DP * DP );
    CD->flConfidence = (float *) cvAlloc( sizeof( float ) * SamplesNum );
    CD->flCumulative = (float *) cvAlloc( sizeof( float ) * SamplesNum );

    CD->RandS        = (CvRandState *) cvAlloc( sizeof( CvRandState ) * DP );
    CD->Temp         = (float *) cvAlloc( sizeof( float ) * DP );
    CD->RandomSample = (float *) cvAlloc( sizeof( float ) * DP );

    /* Returning created structure */
    return CD;
}

//*************************************************************
// Condensation構造体を開放する関数
//*************************************************************
void myReleaseConDensation( CvConDensation ** ConDensation )
{
    CvConDensation *CD = *ConDensation;
    
    if( !ConDensation ) { printf("Condensation Error.\n"); return; }

    /* freeing the memory */
	cvFree( (void**)&CD->State );
    cvFree( (void**)&CD->DynamMatr);
    cvFree( (void**)&CD->flConfidence );
    cvFree( (void**)&CD->flCumulative );
    cvFree( (void**)&CD->flSamples[0] );
    cvFree( (void**)&CD->flNewSamples[0] );
    cvFree( (void**)&CD->flSamples );
    cvFree( (void**)&CD->flNewSamples );
    cvFree( (void**)&CD->Temp );
    cvFree( (void**)&CD->RandS );
    cvFree( (void**)&CD->RandomSample );
    /* release structure */
    cvFree( (void**)ConDensation );
}

//*************************************************************
// 期待値の計算を行う関数
//*************************************************************
void myConDensUpdateByTime( CvConDensation * ConDens )
{
    if( !ConDens ) { printf("Condensation Error.\n"); return; }

    /* Sets Temp to Zero */
    icvSetZero_32f( ConDens->Temp, ConDens->DP, 1 );

	// 尤度で重み付けされたサンプルの値の和：ConDens->Temp
    /* Calculating the Mean */
    float Sum = 0.000001f;//floatの有効桁数は7なので，0に近い最小値
    for(int i = 0; i < ConDens->SamplesNum; i++ ){
        //低すぎる期待値のサンプルは期待値計算に参加しない
        //if(ConDens->flConfidence[i]<1.0e-3) continue;

		//サンプルを尤度で重み付けして一時的にConDens->Stateへ
        icvScaleVector_32f( ConDens->flSamples[i], ConDens->State, ConDens->DP, ConDens->flConfidence[i] );
		//ConDens->Tempに累積する
        icvAddVector_32f( ConDens->Temp, ConDens->State, ConDens->Temp, ConDens->DP );
		//尤度をSumへ累積する
        Sum += ConDens->flConfidence[i];
		//サンプリングのため累積尤度を各サンプルにセット
        ConDens->flCumulative[i] = Sum;
    }
	// 尤度で重み付けされたサンプルの値の和を尤度の和で割って正規化：ConDens->State
    icvScaleVector_32f( ConDens->Temp, ConDens->State, ConDens->DP, 1.f / Sum );
	// 合計尤度をMPに保存(本来MPは観測次元数を表すものだが、使われていないためここで使用)
	ConDens->MP = (int)Sum;

}

//*************************************************************
// ばらつきの再設定を行う関数
//*************************************************************
void myConDensUpdateDeviation( CvConDensation * conDens, CvMat * mean, CvMat * deviation )
{
	if( !conDens || !deviation || !mean )                                             { printf("Condensation Error.\n"); return; }
    if( CV_MAT_TYPE(deviation->type) != CV_32FC1 || !CV_ARE_TYPES_EQ(deviation,mean) ){ printf("Condensation Error.\n"); return; }
    if( (deviation->cols != 1) || (mean->cols != 1) )                                 { printf("Condensation Error.\n"); return; }
    if( (deviation->rows != conDens->DP) || (mean->rows != conDens->DP) )             { printf("Condensation Error.\n"); return; }

	float *StdDev  = deviation->data.fl;
    float *MeanVal = mean->data.fl;
    for(int i = 0; i < conDens->DP; i++ ){
		cvRandInit( &(conDens->RandS[i]), MeanVal[i], StdDev[i], i, CV_RAND_NORMAL );
	}

}

//*************************************************************
// サンプルの更新を行う関数
//*************************************************************
void myConDensUpdateSample( CvConDensation * ConDens )
{
	///// サンプリング //////////////////////////////////////////////////////////////////////////////////
	// 平均尤度を計算(一様に尤度を分割してサンプリングするため)
    float Sum = 0;
    Sum = (float)ConDens->MP / ConDens->SamplesNum;
    /* Updating the set of random samples */
	// 尤度の比に従ってサンプルを取り出す
    for(int i = 0; i < ConDens->SamplesNum; i++ ){
        int j = 1;
        while( (ConDens->flCumulative[j] <= (float) i * Sum + 0.000001f) && (j < (ConDens->SamplesNum-1)) )
		{
            j++;
        }
        icvCopyVector_32f( ConDens->flSamples[j], ConDens->DP, ConDens->flNewSamples[i] );
    }

	///// サンプル毎にディフューズを加える //////////////////////////////////////////////////////////
    /* Adding the random-generated vector to every vector in sample set */
    for( int i = 0; i < ConDens->SamplesNum; i++ ){
		// ガウシアンノイズ生成
        for(int j = 0; j < ConDens->DP; j++ ){
            cvbRand( ConDens->RandS + j, ConDens->RandomSample + j, 1 );
        }
		// ガウシアンノイズを付与
		icvAddVector_32f( ConDens->flNewSamples[i], ConDens->RandomSample, ConDens->flSamples[i], ConDens->DP );
    }

}

//*************************************************************
// サンプルの初期化関数
//*************************************************************
void myConDensInitSampleSet( CvConDensation * conDens, CvMat * initvalue, CvMat * mean, CvMat * deviation )
{
	if( !conDens || !deviation || !mean || !initvalue ){
		printf("Condensation Error.\n"); return; }
    if( (deviation->cols != 1) || (mean->cols != 1) || (initvalue->cols != 1) ){
		printf("Condensation Error.\n"); return; }
    if( (deviation->rows != conDens->DP) || (mean->rows != conDens->DP) || (initvalue->rows != conDens->DP) ){
		printf("Condensation Error.\n"); return; }
    if( CV_MAT_TYPE(deviation->type) != CV_32FC1 || !CV_ARE_TYPES_EQ(deviation,mean) || !CV_ARE_TYPES_EQ(deviation,initvalue) ){
		printf("Condensation Error.\n"); return; }

    float *StdDev  = deviation->data.fl;
    float *MeanVal = mean->data.fl;
    float *InitVal = initvalue->data.fl;
	float Prob = 1.f / conDens->SamplesNum;

	/* Initializing the structures to create initial Sample set */
    for(int i = 0; i < conDens->DP; i++ ){
        cvRandInit( &(conDens->RandS[i]), MeanVal[i], StdDev[i], i, CV_RAND_NORMAL );
    }
    /* Generating the samples */
	for(int j = 0; j < conDens->SamplesNum; j++ ){
        for(int i = 0; i < conDens->DP; i++ ){
            cvbRand( conDens->RandS + i, conDens->flSamples[j] + i, 1 );
        }
        conDens->flConfidence[j] = Prob;
		icvAddVector_32f( conDens->flSamples[j], InitVal, conDens->flSamples[j], conDens->DP ); // 初期値をプラス
    }

}


double Dispersion_EffectivenessSample(CvConDensation * ConDens,int sum,int count,int DP){//ConDens:サンプル情報,sum:有効サンプルの値の累計値,count:有効サンプル数,DPは求める座標次元(0:x座標,1:y座標,2:角度)
	double mean=0.0,sum_i=0.0,dispersion=0.0;
	int j_before = -1;//前回更新のjを格納
	mean = (double)sum / (double)count;//有効サンプルの指定座標平均を求める
	for(int i=0;i<ConDens->SamplesNum;i++){
		int j = 1;
				while((ConDens->flCumulative[j]<=(double)i*(double)ConDens->MP/300.0+0.000001f)&&(j<(ConDens->SamplesNum-1)))
				{
					j++;
				}
				if(j_before != j){ //更新時
					sum_i = sum_i + (mean - (double)ConDens->flSamples[i][DP])*(mean - (double)ConDens->flSamples[i][DP]);//(有効サンプルの平均と各有効サンプルの座標の差)の2乗を累積
				}
	}
	dispersion = sum_i/(double)count;//有効サンプル数で割ることでそのサイクルの有効サンプルの平均分散を求める
	return dispersion;
}