#include "MyEllipseNormalEvaluation.h"
#include <vector>
#include <cmath>

// --- 定数 ---
constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 640;
constexpr double CONTOUR_PI = 3.14159265358979323846;
constexpr int OUT_OF_IMAGE_MARGIN = 10;
constexpr float EPS = 1.0e-7f;

// --- グローバル変数 (モジュール内) ---
namespace {
    cv::Point g_sensorPos;

    // 胴体モデル
    int g_bodyMajorAxis = 0;
    int g_bodyMinorAxis = 0;
    std::vector<cv::Point> g_bodyContourPositionTable;
    std::vector<cv::Point> g_bodyNormalVectorTable;

    // 頭部モデル
    int g_headAxis = 0;
    std::vector<cv::Point> g_headContourPositionTable;
    std::vector<cv::Point> g_headNormalVectorTable;

    // ヘルパー関数プロトタイプ
    int CalculateBodyContourPosition(const cv::Point& center, int rotation, std::vector<int>& bodyEnableFlg, std::vector<cv::Point>& bodyPosData);
    int CalculateHeadContourPosition(const cv::Point& center, const cv::Size& size, std::vector<int>& headEnableFlg, std::vector<cv::Point>& headPosData);
}

// --- 関数実装 ---

void SetSensorPosition(const cv::Point& sensorPos) {
    g_sensorPos = sensorPos;
}

void StoreBodyContourPosition(int majorAxis, int minorAxis, int stepTheta) {
    g_bodyMajorAxis = majorAxis;
    g_bodyMinorAxis = minorAxis;

    g_bodyContourPositionTable.clear();
    g_bodyNormalVectorTable.clear();

    if (stepTheta <= 0) return;

    for (int i = 0; i < 360 / stepTheta; ++i) {
        double angleRad = i * stepTheta * CONTOUR_PI / 180.0 + CONTOUR_PI;
        // 胴体輪郭位置
        cv::Point pos(
            cvRound(majorAxis * cos(angleRad)),
            cvRound(minorAxis * sin(angleRad))
        );
        g_bodyContourPositionTable.push_back(pos);

        // 胴体法線ベクトル
        cv::Point vec(
            (long long)minorAxis * minorAxis * pos.x,
            (long long)majorAxis * majorAxis * pos.y
        );
        g_bodyNormalVectorTable.push_back(vec);
    }
}

void StoreHeadContourPosition(int headAxis, int stepTheta) {
    g_headAxis = headAxis;

    g_headContourPositionTable.clear();
    g_headNormalVectorTable.clear();

    if (stepTheta <= 0) return;

    for (int i = 0; i < 360 / stepTheta; ++i) {
        double angleRad = i * stepTheta * CONTOUR_PI / 180.0 + CONTOUR_PI;
        // 頭部輪郭位置
        cv::Point pos(
            cvRound(headAxis * cos(angleRad)),
            cvRound(headAxis * sin(angleRad))
        );
        g_headContourPositionTable.push_back(pos);
        // 頭部法線ベクトル (円なので位置ベクトルと同じ)
        g_headNormalVectorTable.push_back(pos);
    }
}

float CalculateBodyLikelihood(const cv::Mat& distTransformedImg, const cv::Point& center, int rotation) {
    std::vector<cv::Point> bodyPositionData(g_bodyContourPositionTable.size());
    std::vector<int> bodyEnableFlg(g_bodyContourPositionTable.size());
    
    CalculateBodyContourPosition(center, rotation, bodyEnableFlg, bodyPositionData);

    float max_dist_val = 0.0f;
    int evalCnt = 0;
    cv::Point prevPosition(-1, -1);

    for (size_t i = 0; i < bodyPositionData.size(); ++i) {
        const auto& pos = bodyPositionData[i];

        // 同じピクセルを複数回評価しない
        if (pos == prevPosition) continue;
        prevPosition = pos;

        // 画像範囲内で、かつセンサから見える方向の点のみ評価
        if (pos.x > OUT_OF_IMAGE_MARGIN && pos.x < IMAGE_WIDTH - OUT_OF_IMAGE_MARGIN &&
            pos.y > OUT_OF_IMAGE_MARGIN && pos.y < IMAGE_HEIGHT - OUT_OF_IMAGE_MARGIN) {
            
            if (bodyEnableFlg[i] == 1) {
                // 距離変換画像の値を取得 (CV_32F)
                float dist_val = distTransformedImg.at<float>(pos.y, pos.x);
                if (dist_val > max_dist_val) {
                    max_dist_val = dist_val;
                }
                evalCnt++;
            }
        }
    }

    // 元のコードの尤度計算式を再現
    double likelihood = exp(-(max_dist_val * max_dist_val) / 5.0) * exp((double)evalCnt);
    return max_dist_val > EPS ? static_cast<float>(likelihood) : 0.0f;
}

void DrawBodyContour(cv::Mat& image, const cv::Point& center, int rotation) {
    // 胴体の輪郭位置の計算と描画
    std::vector<cv::Point> bodyPositionData(g_bodyContourPositionTable.size());
    std::vector<int> bodyEnableFlg(g_bodyContourPositionTable.size());
    CalculateBodyContourPosition(center, rotation, bodyEnableFlg, bodyPositionData);

    for (size_t i = 0; i < bodyPositionData.size(); ++i) {
        cv::Scalar color = (bodyEnableFlg[i] == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128);
        cv::circle(image, bodyPositionData[i], 2, color, 1, cv::LINE_AA);
    }

    // 頭部の輪郭位置の計算と描画
    std::vector<cv::Point> headPositionData(g_headContourPositionTable.size());
    std::vector<int> headEnableFlg(g_headContourPositionTable.size());
    CalculateHeadContourPosition(center, cv::Size(g_headAxis, g_headAxis), headEnableFlg, headPositionData);

    for (size_t j = 0; j < headPositionData.size(); ++j) {
        cv::Scalar color = (headEnableFlg[j] == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128);
        cv::circle(image, headPositionData[j], 2, color, 1, cv::LINE_AA);
    }
}


// --- 静的ヘルパー関数の実装 ---
namespace {
    int CalculateBodyContourPosition(const cv::Point& center, int rotation, std::vector<int>& bodyEnableFlg, std::vector<cv::Point>& bodyPosData) {
        double angleRad = rotation * CONTOUR_PI / 180.0;
        double cos_r = cos(angleRad);
        double sin_r = sin(angleRad);
        int NofBodyEnablePos = 0;

        for (size_t i = 0; i < g_bodyContourPositionTable.size(); ++i) {
            const auto& tablePos = g_bodyContourPositionTable[i];
            const auto& tableVec = g_bodyNormalVectorTable[i];

            // 胴体の回転と並行移動
            cv::Point contourPos(
                cvRound(cos_r * tablePos.x - sin_r * tablePos.y) + center.x,
                cvRound(sin_r * tablePos.x + cos_r * tablePos.y) + center.y
            );
            bodyPosData[i] = contourPos;

            cv::Point contourVec(
                cvRound(cos_r * tableVec.x - sin_r * tableVec.y),
                cvRound(sin_r * tableVec.x + cos_r * tableVec.y)
            );

            // 法線ベクトルとの内積をチェック
            long long innerProduct = (long long)contourVec.x * (g_sensorPos.x - contourPos.x) + (long long)contourVec.y * (g_sensorPos.y - contourPos.y);
            double mag = std::sqrt((double)cv::norm(g_sensorPos - contourPos) * cv::norm(contourVec));
            
            if (mag > EPS && (innerProduct / mag > 0.2)) {
                bodyEnableFlg[i] = 1;
                NofBodyEnablePos++;
            } else {
                bodyEnableFlg[i] = 0;
            }
        }
        return NofBodyEnablePos;
    }

    int CalculateHeadContourPosition(const cv::Point& center, const cv::Size& size, std::vector<int>& headEnableFlg, std::vector<cv::Point>& headPosData) {
        int NofHeadEnablePos = 0;
        double scale_x = (g_headAxis > 0) ? (double)size.width / g_headAxis : 1.0;
        double scale_y = (g_headAxis > 0) ? (double)size.height / g_headAxis : 1.0;

        for (size_t i = 0; i < g_headContourPositionTable.size(); ++i) {
             const auto& tablePos = g_headContourPositionTable[i];
             const auto& tableVec = g_headNormalVectorTable[i];

            // サイズ変更と並行移動
            cv::Point contourPos(
                cvRound(scale_x * tablePos.x) + center.x,
                cvRound(scale_y * tablePos.y) + center.y
            );
            headPosData[i] = contourPos;

            cv::Point contourVec(
                cvRound(scale_x * tableVec.x),
                cvRound(scale_y * tableVec.y)
            );
            
            // 法線ベクトルとの内積をチェック
            long long innerProduct = (long long)contourVec.x * (g_sensorPos.x - contourPos.x) + (long long)contourVec.y * (g_sensorPos.y - contourPos.y);
            double mag = std::sqrt((double)cv::norm(g_sensorPos - contourPos) * cv::norm(contourVec));
            
            if (mag > EPS && (innerProduct / mag > -0.2)) {
                headEnableFlg[i] = 1;
                NofHeadEnablePos++;
            } else {
                headEnableFlg[i] = 0;
            }
        }
        return NofHeadEnablePos;
    }
}
