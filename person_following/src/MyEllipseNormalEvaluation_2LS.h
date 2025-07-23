#ifndef __MY_ELLIPSE_NORMAL_EVALUATION_2LS_H__
#define __MY_ELLIPSE_NORMAL_EVALUATION_2LS_H__

#include <opencv2/opencv.hpp>

/**
 * @brief 指定されたインデックスのセンサの位置を設定します。
 * @param index センサのインデックス (0 or 1)
 * @param sensorPos 画像座標系におけるセンサの位置
 */
void SetSensorPosition_2LS(int index, const cv::Point& sensorPos);

/**
 * @brief 指定されたインデックスのセンサの可視範囲マスクを設定します。
 * @param index センサのインデックス (0 or 1)
 * @param visibleRangeMask 他方のセンサによって遮られない領域を示すマスク画像 (CV_8UC1, 0でない値が可視)
 */
void SetSensorVisibleRange_2LS(int index, const cv::Mat& visibleRangeMask);

/**
 * @brief 追跡対象の胴体モデル（楕円）の輪郭点と法線ベクトルのテーブルを作成します。
 * @param majorAxis 楕円の長軸半径（ピクセル）
 * @param minorAxis 楕円の短軸半径（ピクセル）
 * @param stepTheta 輪郭点を生成する角度の刻み（度）
 */
void StoreBodyContourPosition_2LS(int majorAxis, int minorAxis, int stepTheta);

/**
 * @brief 追跡対象の頭部モデル（円）の輪郭点と法線ベクトルのテーブルを作成します。
 * @param headAxis 円の半径（ピクセル）
 * @param stepTheta 輪郭点を生成する角度の刻み（度）
 */
void StoreHeadContourPosition_2LS(int headAxis, int stepTheta);

/**
 * @brief 距離変換画像を用いて、指定された状態（位置、角度）の尤度を計算します。
 * 2つのセンサの可視範囲を考慮します。
 * @param distTransformedImg 距離変換された画像 (CV_32FC1)
 * @param center 評価する楕円の中心座標
 * @param rotation 評価する楕円の回転角度（度）
 * @return 計算された尤度
 */
float CalculateBodyLikelihood_2LS(const cv::Mat& distTransformedImg, const cv::Point& center, int rotation);

/**
 * @brief 指定された画像に、推定された胴体と頭部の輪郭を描画します。
 * 2つのセンサの可視範囲に応じて色分けします。
 * @param image 描画対象の画像 (CV_8UC3)
 * @param center 描画する楕円の中心座標
 * @param rotation 描画する楕円の回転角度（度）
 */
void DrawBodyContour_2LS(cv::Mat& image, const cv::Point& center, int rotation);

#endif // __MY_ELLIPSE_NORMAL_EVALUATION_2LS_H__
