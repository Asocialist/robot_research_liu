/*
* MyCondensation.h
* (Rewritten for modern C++ and OpenCV 4)
*/
#ifndef __MYCONSDENSATION_H__
#define __MYCONSDENSATION_H__

#include <opencv2/opencv.hpp>
#include <vector>

// 用现代C++结构体替换旧的 CvConDensation
struct MyCondensation {
    int state_dimensions;
    int samples_num;
    
    cv::Mat state; // 状态向量 (DP x 1)
    std::vector<cv::Mat> samples; // 粒子/样本 (每个都是 DP x 1 的 Mat)
    cv::Mat confidence; // 置信度/权重 (1 x SamplesNum)
    cv::RNG rng; // OpenCV的现代随机数生成器
};

// 更新函数签名以使用新的结构体和cv::Mat
MyCondensation* myCreateConDensation(int DP, int SamplesNum);
void myReleaseConDensation(MyCondensation** cd);
void myConDensInitSampleSet(MyCondensation* cd, const cv::Mat& init_value, const cv::Mat& mean, const cv::Mat& deviation);
void myConDensUpdateByTime(MyCondensation* cd);
void myConDensUpdateSample(MyCondensation* cd);

#endif //__MYCONSDENSATION_H__