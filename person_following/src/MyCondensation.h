// MyCondensation.h (OpenCV 4向けに修正)
#ifndef __MYCONSDENSATION_H__
#define __MYCONSDENSATION_H__

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief Condensation (Particle Filter) アルゴリズムをOpenCV 4 C++ APIで実装したクラス
 *
 * 元のC APIベースのコードをOpenCV 4で動作するように移植したものです。
 * APIの互換性をある程度維持しつつ、C++の機能 (cv::Mat, std::vector) を使用しています。
 */
class MyCondensation {
public:
    /**
     * @brief コンストラクタ
     * @param dp 状態ベクトルの次元数
     * @param samplesNum パーティクル（サンプル）の数
     */
    MyCondensation(int dp, int samplesNum);

    /**
     * @brief デストラクタ
     */
    ~MyCondensation();

    /**
     * @brief サンプルセットを初期化します。
     * * 各サンプルは initvalue を中心とした正規分布に従って生成されます。
     * @param initvalue 初期状態ベクトル (dp x 1, CV_32F)
     * @param mean ノイズの平均 (dp x 1, CV_32F) - 通常はゼロベクトル
     * @param deviation ノイズの標準偏差 (dp x 1, CV_32F)
     */
    void initSampleSet(const cv::Mat& initvalue, const cv::Mat& mean, const cv::Mat& deviation);

    /**
     * @brief 重みに基づいて状態の期待値を計算します。
     * * 計算後、リサンプリングのために内部の累積重みテーブルも更新します。
     */
    void updateByTime();

    /**
     * @brief サンプルを更新します (リサンプリング + 拡散)。
     * * 最初に重みに基づいてリサンプリングを行い、その後、
     * updateDeviationで設定されたノイズを加えてサンプルを拡散させます。
     */
    void updateSample();
    
    /**
     * @brief 拡散ステップで使われるノイズの標準偏差を設定します。
     * * この関数は updateSample の前に呼ばれる必要があります。
     * @param mean ノイズの平均 (dp x 1, CV_32F) - 現在の実装では未使用 (平均0を仮定)
     * @param deviation ノイズの標準偏差 (dp x 1, CV_32F)
     */
    void updateDeviation(const cv::Mat& mean, const cv::Mat& deviation);

    // --- Public Member Variables ---
    
    cv::Mat state;              ///< 推定された状態ベクトル (dp x 1)
    std::vector<float> confidence; ///< 各サンプルの重み（信頼度）の配列 (サイズ: samplesNum)
    std::vector<cv::Mat> samples;  ///< サンプル（パーティクル）の配列 (各要素が dp x 1 の cv::Mat)
    float totalConfidence;      ///< 正規化前の重みの合計 (元のコードのMPに相当)

private:
    int dp;                     ///< 状態ベクトルの次元
    int samplesNum;             ///< サンプル数
    
    std::vector<float> cumulative; ///< 重みの累積値 (リサンプリング用)
    std::vector<cv::Mat> newSamples; ///< リサンプリング後の一時的なサンプル配列
    
    cv::Mat noiseDeviation;     ///< 拡散用ノイズの標準偏差
    cv::RNG rng;                ///< 乱数生成器
};

#endif //__MYCONSDENSATION_H__
