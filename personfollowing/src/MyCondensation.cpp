/*
* MyCondensation.cpp
* (Rewritten for modern C++ and OpenCV 4)
*/
#include "MyCondensation.h"

using namespace cv;

MyCondensation* myCreateConDensation(int DP, int SamplesNum) {
    if (DP <= 0 || SamplesNum <= 0) return nullptr;

    MyCondensation* cd = new MyCondensation();
    cd->state_dimensions = DP;
    cd->samples_num = SamplesNum;
    
    cd->state = Mat::zeros(DP, 1, CV_32F);
    cd->confidence = Mat::zeros(1, SamplesNum, CV_32F);

    cd->samples.resize(SamplesNum);
    for (int i = 0; i < SamplesNum; ++i) {
        cd->samples[i] = Mat::zeros(DP, 1, CV_32F);
    }
    
    // 使用当前时间初始化随机数生成器
    cd->rng = RNG(getTickCount());

    return cd;
}

void myReleaseConDensation(MyCondensation** cd) {
    if (cd && *cd) {
        delete *cd;
        *cd = nullptr;
    }
}

void myConDensInitSampleSet(MyCondensation* cd, const Mat& init_value, const Mat& mean, const Mat& deviation) {
    if (!cd) return;

    for (int i = 0; i < cd->samples_num; ++i) {
        // Create the matrix to hold our random numbers
        Mat random_sample(cd->state_dimensions, 1, CV_32F);
        
        // Loop through each dimension (x, y, angle)
        for (int j = 0; j < cd->state_dimensions; ++j) {
            // Get the specific mean and standard deviation for this dimension
            float m = mean.at<float>(j);
            float d = deviation.at<float>(j);
            // Generate one Gaussian random number and place it in the matrix
            random_sample.at<float>(j) = cd->rng.gaussian(d) + m;
        }
        
        // Initialize the sample = initial value + our manually generated noise
        cd->samples[i] = init_value + random_sample;
        cd->confidence.at<float>(0, i) = 1.0f / cd->samples_num;
    }
}

void myConDensUpdateByTime(MyCondensation* cd) {
    if (!cd) return;

    Mat weighted_sum = Mat::zeros(cd->state_dimensions, 1, CV_32F);
    float confidence_sum = 0.0f;
    
    Mat cumulative_confidence = Mat::zeros(1, cd->samples_num, CV_32F);

    for (int i = 0; i < cd->samples_num; ++i) {
        float conf = cd->confidence.at<float>(0, i);
        weighted_sum += cd->samples[i] * conf;
        confidence_sum += conf;
        cumulative_confidence.at<float>(0, i) = confidence_sum;
    }

    if (confidence_sum > 1e-9) {
        cd->state = weighted_sum / confidence_sum;
    }
    
    // --- 重采样 (Resampling) ---
    std::vector<Mat> new_samples;
    new_samples.resize(cd->samples_num);
    
    float step = confidence_sum / cd->samples_num;
    float random_start = cd->rng.uniform(0.0f, step);

    int current_sample_idx = 0;
    for (int i = 0; i < cd->samples_num; ++i) {
        float target_confidence = random_start + i * step;
        while (cumulative_confidence.at<float>(0, current_sample_idx) < target_confidence) {
            current_sample_idx++;
            if (current_sample_idx >= cd->samples_num) {
                current_sample_idx = cd->samples_num - 1;
                break;
            }
        }
        new_samples[i] = cd->samples[current_sample_idx].clone();
    }
    
    cd->samples = new_samples;
}

void myConDensUpdateSample(MyCondensation* cd) {
    if (!cd) return;
    
    // Define the parameters for the random walk
    Mat mean = Mat::zeros(cd->state_dimensions, 1, CV_32F);
    Mat stddev = Mat::ones(cd->state_dimensions, 1, CV_32F) * 2.0f;

    for(int i = 0; i < cd->samples_num; ++i) {
        // We will fill the random_noise matrix manually
        Mat random_noise(cd->state_dimensions, 1, CV_32F);

        for (int j = 0; j < cd->state_dimensions; ++j) {
            float m = mean.at<float>(j);
            float d = stddev.at<float>(j);
            random_noise.at<float>(j) = cd->rng.gaussian(d) + m;
        }

        cd->samples[i] += random_noise;
    }
}