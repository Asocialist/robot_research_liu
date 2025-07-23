#include "MyEllipseNormalEvaluation_2LS.h"
#include <vector>
#include <cmath>

// --- 定数 ---
constexpr double CONTOUR_PI = 3.14159265358979323846;
constexpr float EPS = 1.0e-7f;
constexpr int OUT_OF_IMAGE_MARGIN = 10;

// --- グローバル変数 (モジュール内スコープ) ---
namespace {
    cv::Point g_sensorPos[2];
    cv::Mat   g_visibleMask[2];

    // 胴体モデル
    int m_NofBodyEdgePos = 0;
    int m_bodyMajorAxis  = 0;
    int m_bodyMinorAxis  = 0;
    std::vector<cv::Point> g_bodyContourPositionTable;
    std::vector<cv::Point> g_bodyNormalVectorTable;

    // 頭部モデル
    int m_NofHeadEdgePos = 0;
    int m_headAxis       = 0;
    std::vector<cv::Point> g_headContourPositionTable;
    std::vector<cv::Point> g_headNormalVectorTable;

    // ヘルパー関数プロトタイプ
    int CalculateBodyContourPosition_Helper(const cv::Point& center, int rotation, std::vector<int>& bodyEnableFlg, std::vector<cv::Point>& bodyPosData);
    int CalculateHeadContourPosition_Helper(const cv::Point& center, const cv::Size& size, std::vector<int>& headEnableFlg, std::vector<cv::Point>& headPosData);
}

// --- 関数実装 ---

void SetSensorPosition_2LS(int index, const cv::Point& sensorPos) {
    if (index < 0 || index >= 2) {
        fprintf(stderr, "SetSensorPosition_2LS: illegal sensorID: %d\n", index);
        return;
    }
    g_sensorPos[index] = sensorPos;
}

void SetSensorVisibleRange_2LS(int index, const cv::Mat& visibleRangeMask) {
    if (index < 0 || index >= 2) {
        fprintf(stderr, "SetSensorVisibleRange_2LS: illegal sensorID: %d\n", index);
        return;
    }
    if (visibleRangeMask.type() != CV_8UC1) {
        fprintf(stderr, "SetSensorVisibleRange_2LS: mask must be CV_8UC1\n");
        return;
    }
    g_visibleMask[index] = visibleRangeMask.clone();
}

void StoreBodyContourPosition_2LS(int majorAxis, int minorAxis, int stepTheta) {
    m_bodyMajorAxis = majorAxis;
    m_bodyMinorAxis = minorAxis;
    g_bodyContourPositionTable.clear();
    g_bodyNormalVectorTable.clear();
    m_NofBodyEdgePos = 0;

    if (stepTheta <= 0) return;

    for (int i = 0; i < 360 / stepTheta; ++i) {
        double angleRad = i * stepTheta * CONTOUR_PI / 180.0 + CONTOUR_PI;
        cv::Point pos(cvRound(majorAxis * cos(angleRad)), cvRound(minorAxis * sin(angleRad)));
        g_bodyContourPositionTable.push_back(pos);
        
        cv::Point vec((long long)minorAxis * minorAxis * pos.x, (long long)majorAxis * majorAxis * pos.y);
        g_bodyNormalVectorTable.push_back(vec);
        m_NofBodyEdgePos++;
    }
}

void StoreHeadContourPosition_2LS(int headAxis, int stepTheta) {
    m_headAxis = headAxis;
    g_headContourPositionTable.clear();
    g_headNormalVectorTable.clear();
    m_NofHeadEdgePos = 0;

    if (stepTheta <= 0) return;

    for (int i = 0; i < 360 / stepTheta; ++i) {
        double angleRad = i * stepTheta * CONTOUR_PI / 180.0 + CONTOUR_PI;
        cv::Point pos(cvRound(headAxis * cos(angleRad)), cvRound(headAxis * sin(angleRad)));
        g_headContourPositionTable.push_back(pos);
        g_headNormalVectorTable.push_back(pos); // 円の法線は位置ベクトルと同じ
        m_NofHeadEdgePos++;
    }
}


float CalculateBodyLikelihood_2LS(const cv::Mat& urgDstImg, const cv::Point& center, int rotation) {
    if (g_bodyContourPositionTable.empty()) return 0.0f;

    std::vector<cv::Point> bodyPositionData(m_NofBodyEdgePos);
    std::vector<int> bodyEnableFlg(m_NofBodyEdgePos);
    CalculateBodyContourPosition_Helper(center, rotation, bodyEnableFlg, bodyPositionData);

    float max_dist_val = 0.0f;
    int evalCnt = 0;
    cv::Point prevPosition(-1, -1);

    for (int i = 0; i < m_NofBodyEdgePos; ++i) {
        const auto& pos = bodyPositionData[i];
        if (pos == prevPosition) continue;
        prevPosition = pos;

        if (pos.x > OUT_OF_IMAGE_MARGIN && pos.x < urgDstImg.cols - OUT_OF_IMAGE_MARGIN &&
            pos.y > OUT_OF_IMAGE_MARGIN && pos.y < urgDstImg.rows - OUT_OF_IMAGE_MARGIN) {
            
            if (bodyEnableFlg[i] == 1) {
                float tmp = urgDstImg.at<float>(pos);
                if (tmp > max_dist_val) max_dist_val = tmp;
                evalCnt++;
            }
        }
    }

    double ret = exp(-(double)(max_dist_val * max_dist_val) / 5.0) * exp((double)evalCnt);
    return max_dist_val > EPS ? static_cast<float>(ret) : 0.0f;
}
   
void DrawBodyContour_2LS(cv::Mat& image, const cv::Point& center, int rotation) {
    if (g_bodyContourPositionTable.empty() || g_headContourPositionTable.empty()) return;

    std::vector<cv::Point> bodyPositionData(m_NofBodyEdgePos);
    std::vector<int> bodyEnableFlg(m_NofBodyEdgePos);
    CalculateBodyContourPosition_Helper(center, rotation, bodyEnableFlg, bodyPositionData);

    for (int i = 0; i < m_NofBodyEdgePos; ++i) {
        cv::Scalar color = (bodyEnableFlg[i] == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128);
        cv::circle(image, bodyPositionData[i], 2, color, -1);
    }
    
    std::vector<cv::Point> headPositionData(m_NofHeadEdgePos);
    std::vector<int> headEnableFlg(m_NofHeadEdgePos);
    CalculateHeadContourPosition_Helper(center, cv::Size(m_headAxis, m_headAxis), headEnableFlg, headPositionData);
    
    for (int j = 0; j < m_NofHeadEdgePos; ++j) {
        cv::Scalar color = (headEnableFlg[j] == 1) ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
        cv::circle(image, headPositionData[j], 2, color, -1);
    }
}


namespace { // ヘルパー関数の実装
    int getVisibleSensorID(const cv::Point& pos) {
        if (!g_visibleMask[0].empty() && g_visibleMask[0].at<uchar>(pos) != 0) return 0;
        if (!g_visibleMask[1].empty() && g_visibleMask[1].at<uchar>(pos) != 0) return 1;
        return -1; // どちらの専用領域でもない (重なり領域など)
    }

    int CalculateBodyContourPosition_Helper(const cv::Point& center, int rotation, std::vector<int>& bodyEnableFlg, std::vector<cv::Point>& bodyPosData) {
        int nofBodyEnablePos = 0;
        double angleRad = rotation * CONTOUR_PI / 180.0;
        double cos_r = cos(angleRad);
        double sin_r = sin(angleRad);

        int centerSensorId = getVisibleSensorID(center);
        if (centerSensorId == -1) centerSensorId = 0; // デフォルト

        for (int i = 0; i < m_NofBodyEdgePos; ++i) {
            const auto& tablePos = g_bodyContourPositionTable[i];
            const auto& tableVec = g_bodyNormalVectorTable[i];

            cv::Point contourPos(
                cvRound(cos_r * tablePos.x - sin_r * tablePos.y) + center.x,
                cvRound(sin_r * tablePos.x + cos_r * tablePos.y) + center.y
            );
            bodyPosData[i] = contourPos;

            cv::Point contourVec(
                cvRound(cos_r * tableVec.x - sin_r * tableVec.y),
                cvRound(sin_r * tableVec.x + cos_r * tableVec.y)
            );

            long long innerProduct = (long long)contourVec.x * (g_sensorPos[centerSensorId].x - contourPos.x) + (long long)contourVec.y * (g_sensorPos[centerSensorId].y - contourPos.y);
            double mag = cv::norm(g_sensorPos[centerSensorId] - contourPos) * cv::norm(contourVec);
            
            if (mag > EPS && (innerProduct / mag > 0.2)) {
                bodyEnableFlg[i] = 1;
                nofBodyEnablePos++;
            } else {
                bodyEnableFlg[i] = 0;
            }
        }
        return nofBodyEnablePos;
    }

    int CalculateHeadContourPosition_Helper(const cv::Point& center, const cv::Size& size, std::vector<int>& headEnableFlg, std::vector<cv::Point>& headPosData) {
        int nofHeadEnablePos = 0;
        double scale_x = (m_headAxis > 0) ? (double)size.width / m_headAxis : 1.0;
        double scale_y = (m_headAxis > 0) ? (double)size.height / m_headAxis : 1.0;

        int centerSensorId = getVisibleSensorID(center);
        if (centerSensorId == -1) centerSensorId = 0; // デフォルト

        for (int i = 0; i < m_NofHeadEdgePos; ++i) {
            const auto& tablePos = g_headContourPositionTable[i];
            const auto& tableVec = g_headNormalVectorTable[i];

            cv::Point contourPos(
                cvRound(scale_x * tablePos.x) + center.x,
                cvRound(scale_y * tablePos.y) + center.y
            );
            headPosData[i] = contourPos;

            cv::Point contourVec(cvRound(scale_x * tableVec.x), cvRound(scale_y * tableVec.y));

            long long innerProduct = (long long)contourVec.x * (g_sensorPos[centerSensorId].x - contourPos.x) + (long long)contourVec.y * (g_sensorPos[centerSensorId].y - contourPos.y);
            double mag = cv::norm(g_sensorPos[centerSensorId] - contourPos) * cv::norm(contourVec);
            
            if (mag > EPS && (innerProduct / mag > -0.2)) {
                headEnableFlg[i] = 1;
                nofHeadEnablePos++;
            } else {
                headEnableFlg[i] = 0;
            }
        }
        return nofHeadEnablePos;
    }
}
