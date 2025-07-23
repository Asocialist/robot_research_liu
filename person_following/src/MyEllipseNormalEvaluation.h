#ifndef __MY_ELLIPSE_NORMAL_EVALUATION_H__
#define __MY_ELLIPSE_NORMAL_EVALUATION_H__

#include <opencv2/opencv.hpp>

/**
 * @brief 評価で使用するセンサの位置を設定します。
 * @param sensorPos 画像座標系におけるセンサの位置
 */
void SetSensorPosition(const cv::Point& sensorPos);

/**
 * @brief 追跡対象の胴体モデル（楕円）の輪郭点と法線ベクトルのテーブルを作成します。
 * @param majorAxis 楕円の長軸半径（ピクセル）
 * @param minorAxis 楕円の短軸半径（ピクセル）
 * @param stepTheta 輪郭点を生成する角度の刻み（度）
 */
void StoreBodyContourPosition(int majorAxis, int minorAxis, int stepTheta);

/**
 * @brief 追跡対象の頭部モデル（円）の輪郭点と法線ベクトルのテーブルを作成します。
 * @param headAxis 円の半径（ピクセル）
 * @param stepTheta 輪郭点を生成する角度の刻み（度）
 */
void StoreHeadContourPosition(int headAxis, int stepTheta);

/**
 * @brief 距離変換画像を用いて、指定された状態（位置、角度）の尤度を計算します。
 * @param distTransformedImg 距離変換された画像 (CV_32FC1)
 * @param center 評価する楕円の中心座標
 * @param rotation 評価する楕円の回転角度（度）
 * @return 計算された尤度
 */
float CalculateBodyLikelihood(const cv::Mat& distTransformedImg, const cv::Point& center, int rotation);

/**
 * @brief 指定された画像に、推定された胴体と頭部の輪郭を描画します。
 * @param image 描画対象の画像 (CV_8UC3)
 * @param center 描画する楕円の中心座標
 * @param rotation 描画する楕円の回転角度（度）
 */
void DrawBodyContour(cv::Mat& image, const cv::Point& center, int rotation);

#endif // __MY_ELLIPSE_NORMAL_EVALUATION_H__
