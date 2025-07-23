// MyCondensation.cpp (OpenCV 4向けに修正)
#include "MyCondensation.h"
#include <numeric> // std::accumulate
#include <iostream>

/**
 * @brief コンストラクタ
 */
MyCondensation::MyCondensation(int dp, int samplesNum)
    : dp(dp), 
      samplesNum(samplesNum),
      totalConfidence(0.0f),
      rng(cv::getTickCount()) // 現在時刻で乱数生成器を初期化
{
    // 状態ベクトルを初期化
    state = cv::Mat::zeros(dp, 1, CV_32F);

    // サンプルと信頼度を格納するベクターのサイズを確保
    samples.resize(samplesNum);
    newSamples.resize(samplesNum);
    for (int i = 0; i < samplesNum; ++i) {
        samples[i] = cv::Mat::zeros(dp, 1, CV_32F);
        newSamples[i] = cv::Mat::zeros(dp, 1, CV_32F);
    }

    confidence.assign(samplesNum, 0.0f);
    cumulative.assign(samplesNum, 0.0f);

    // ノイズの標準偏差をデフォルトで初期化
    noiseDeviation = cv::Mat::zeros(dp, 1, CV_32F);
}

/**
 * @brief デストラクタ
 */
MyCondensation::~MyCondensation() {
    // cv::Matとstd::vectorは自動的にメモリ解放されるため、何もしなくてよい
}

/**
 * @brief 拡散ステップで使われるノイズの標準偏差を設定
 */
void MyCondensation::updateDeviation(const cv::Mat& mean, const cv::Mat& deviation) {
    if (deviation.empty() || deviation.rows != dp || deviation.cols != 1) {
        std::cerr << "Error: Deviation matrix for updateDeviation is invalid." << std::endl;
        return;
    }
    // `mean` は元のC APIとの互換性のために残されているが、
    // cv::RNG::gaussian(sigma) は平均0の正規分布を生成するため、現在の実装では未使用。
    this->noiseDeviation = deviation.clone();
}

/**
 * @brief サンプルセットを初期化
 */
void MyCondensation::initSampleSet(const cv::Mat& initvalue, const cv::Mat& mean, const cv::Mat& deviation) {
    if (initvalue.empty() || deviation.empty() ||
        initvalue.rows != dp || deviation.rows != dp) {
        std::cerr << "Error: Initialization matrices are invalid." << std::endl;
        return;
    }

    // 拡散用ノイズのパラメータを設定
    updateDeviation(mean, deviation);

    // 均一な重みを設定
    float prob = 1.0f / samplesNum;

    for (int i = 0; i < samplesNum; ++i) {
        // ノイズベクトルを生成
        cv::Mat randomVec(dp, 1, CV_32F);
        for (int j = 0; j < dp; ++j) {
            // updateDeviationで設定された標準偏差で正規乱数を生成
            randomVec.at<float>(j, 0) = rng.gaussian(noiseDeviation.at<float>(j, 0));
        }
        
        // 初期値にノイズを加えてサンプルを生成
        samples[i] = initvalue + randomVec;
        confidence[i] = prob;
    }
}

/**
 * @brief 重みに基づいて状態の期待値を計算
 */
void MyCondensation::updateByTime() {
    // 重みの合計を計算 (正規化のため)
    float sum_confidence = 0.0f;
    for(float conf : confidence) {
        sum_confidence += conf;
    }

    // ゼロ除算を避ける
    if (sum_confidence < 1e-9f) {
        state.setTo(cv::Scalar(0));
        totalConfidence = 0;
        // 累積重みもリセット
        std::fill(cumulative.begin(), cumulative.end(), 0.0f);
        return;
    }

    // 重み付き平均を計算して、新しい状態を推定
    cv::Mat tempState = cv::Mat::zeros(dp, 1, CV_32F);
    for (int i = 0; i < samplesNum; i++) {
        // 重みを正規化しながら加算
        tempState += samples[i] * (confidence[i] / sum_confidence);
    }
    state = tempState.clone();

    // リサンプリングのために累積重みを計算
    cumulative[0] = confidence[0];
    for (int i = 1; i < samplesNum; i++) {
        cumulative[i] = cumulative[i - 1] + confidence[i];
    }
    
    // 元のコードのMPに相当する、正規化されていない重みの合計を保存
    totalConfidence = cumulative.back();
}

/**
 * @brief サンプルを更新 (リサンプリング + 拡散)
 */
void MyCondensation::updateSample() {
    // --- 1. リサンプリング (Stochastic Universal Sampling) ---
    float total_conf = cumulative.back();
    
    // 全ての重みがゼロの場合、リサンプリングは不可能
    if (total_conf <= 1e-9f) {
        // この場合、サンプルはそのまま維持する (拡散のみ後で適用)
        for(int i = 0; i < samplesNum; ++i) {
            newSamples[i] = samples[i].clone();
        }
    } else {
        // 系統リサンプリング（Stochastic Universal Sampling）
        float step = total_conf / samplesNum;
        float start = rng.uniform(0.0f, step);
        
        int cumulative_idx = 0;
        for (int i = 0; i < samplesNum; i++) {
            float pointer = start + i * step;
            
            while (cumulative[cumulative_idx] < pointer) {
                cumulative_idx++;
                // 安全装置
                if (cumulative_idx >= samplesNum) {
                    cumulative_idx = samplesNum - 1;
                    break;
                }
            }
            newSamples[i] = samples[cumulative_idx].clone();
        }
    }

    // --- 2. 拡散 (Diffusion) ---
    if (noiseDeviation.empty() || noiseDeviation.rows != dp) {
        std::cerr << "Error: Noise deviation not set. Call updateDeviation() before updateSample()." << std::endl;
        return;
    }

    for (int i = 0; i < samplesNum; i++) {
        // ノイズベクトルを生成
        cv::Mat randomVec(dp, 1, CV_32F);
        for (int j = 0; j < dp; j++) {
            randomVec.at<float>(j, 0) = rng.gaussian(noiseDeviation.at<float>(j, 0));
        }
        // リサンプリングされたサンプルにノイズを加えて、新しいサンプルセットを生成
        samples[i] = newSamples[i] + randomVec;
    }
}
