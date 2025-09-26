/*!
 * @file EllipseTracker2LS.h
 * @brief 楕円追跡パーティクルフィルタ（Lidar2つ使う版）
 * @details
 * 2つ使う時はどっちの観測領域に追跡対象が居るかでセンサ位置の評価が変わる．
 * 体がどう見えているかの判定もそれに適応させないといけない（楕円モデルと観測がズレるとマズい）．
 * @author Ryota Suzuki
 * @date 2025/6/16
	\copyright Copyright 2024 Ryota Suzuki. All rights reserved.
	\license This project is released under the MIT License.
*/

#pragma once

#define _USE_MATH_DEFINES
#include <cmath> // SUGGESTION: 使用 cmath 替代 math.h

#include <vector>
#include <memory>
#include <algorithm>
#include <random>
#include <deque> // FIX: 缺少 deque 头文件

#include <opencv2/opencv.hpp>

//楕円追跡パーティクルフィルタ
class EllipseTracker2LS {
public:
	//楕円テンプレートセット
	struct EllipseTemplate {
		//輪郭点
		std::vector< std::vector<cv::Vec2d> > contour;
		//法線ベクトル
		std::vector< std::vector<cv::Vec2d> > norm;

		typedef std::shared_ptr<EllipseTemplate> Ptr;
	};

	typedef std::shared_ptr<EllipseTracker2LS> Ptr;

private:
	//楕円輪郭点数
	const int TMPL_CNUM = 36; //10度刻み
	//状態次元（画素x，画素y，向き度数(右手0，右回り)）
	const int dim = 3;

	bool flg_exp_calced = false;
	std::mt19937 random;

	int ellipse_width = 24;
	int ellipse_height = 10;

	std::vector<cv::Vec2d> lidarPos;
	std::vector<cv::Mat> lidarOccupancy; // FIX: Mat1b is a typedef for Mat_<uchar>, using Mat is more general

public:
	//パーティクル数
	int NP;
	//前状態 (3 x NP)
	cv::Mat1d state_prev;
	//状態 (3 x NP) 画素x，画素y，向き度数(右手0，右回り)
	cv::Mat1d state;
	//状態期待値（最終結果） getPosの実行で更新される
	cv::Vec3d expPos;
	//パーティクルの尤度
	std::vector<double> likelihoods;
	//累積尤度
	std::vector<double> cumlLikelihoods;
	// 画素px:距離mmの縮尺
	double scale = 0.05;

	//トラッカーのid＠ユーザが管理
	int id;

	//楕円テンプレートメモリ
	EllipseTemplate::Ptr ellipse_tmpl;


	/*!
	 * @brief Constructor
	 */
	EllipseTracker2LS(int NP, int ellipse_width, int ellipse_height, const std::vector<cv::Vec2d> &lidarPos, const std::vector<cv::Mat> &lidarOccupancy, EllipseTemplate::Ptr ellipse_tmpl = nullptr)
		: NP(NP), ellipse_width(ellipse_width), ellipse_height(ellipse_height)
		, expPos(0.0, 0.0, 0.0), lidarPos(lidarPos), lidarOccupancy(lidarOccupancy), ellipse_tmpl(ellipse_tmpl)
	{
		id = 0;

		state = cv::Mat1d(dim, NP, 0.0);
		state_prev = cv::Mat1d(dim, NP, 0.0);
		likelihoods.clear(); likelihoods.resize(NP, 0.0);
		cumlLikelihoods.clear(); cumlLikelihoods.resize(NP, 0.0);

		if (!this->ellipse_tmpl) {
			this->ellipse_tmpl.reset(new EllipseTemplate);
			this->ellipse_tmpl->contour.resize(360, std::vector<cv::Vec2d>(TMPL_CNUM, cv::Vec2d(0, 0)));
			this->ellipse_tmpl->norm.resize(360, std::vector<cv::Vec2d>(TMPL_CNUM, cv::Vec2d(0, 0)));
			constructEllipseTmpl();
		}

		random.seed(5489);
	}
	/*!
	 * @brief Destructor
	 */
	~EllipseTracker2LS() {
	}

	/*!
	 * @brief Initialize particles
	 */
	bool init(int initX, int initY, int initAngl) {
		cv::Mat1d ip(3, 1, std::array<double, 3>{(double)initX, (double)initY, (double)initAngl}.data());
		
        #if _OPENMP
        #pragma omp parallel for
        #endif
		for (int i = 0; i < NP; ++i) ip.copyTo(state.col(i));
		
		state.copyTo(state_prev);
		std::fill(likelihoods.begin(), likelihoods.end(), 1.0 / NP);
		cumlLikelihoods[0] = likelihoods[0];
		for (int i = 1; i < NP; ++i) cumlLikelihoods[i] = likelihoods[i] + cumlLikelihoods[i - 1];

		flg_exp_calced = false;
		return true;
	}

	/*!
	 * @brief Get final calculated pose as expectation
	 */
	cv::Vec3d getPos() {
		if (!flg_exp_calced) {
			expPos = cv::Vec3d(0, 0, 0);
			cv::Vec2d av(0, 0);
			for (int i = 0; i < NP; ++i) {
				expPos[0] += likelihoods[i] * state(0, i);
				expPos[1] += likelihoods[i] * state(1, i);
				double t = state(2, i) * M_PI / 180.0;
				av += likelihoods[i] * cv::Vec2d(cos(t), sin(t));
			}
			expPos[2] = atan2(av[1], av[0]) * 180 / M_PI;
			flg_exp_calced = true;
		}
		return expPos;
	}

	/*!
	 * @brief execute tracking for single frame
	 */
	bool next(cv::Mat lidarDistImage) {
		CV_Assert(lidarDistImage.channels() == 1 && lidarDistImage.depth() == CV_32F);
		flg_exp_calced = false;
		resample();
		update();
		calcWeight(lidarDistImage);
		return true;
	}

private:
	void resample() {
		state.copyTo(state_prev);
		double maxLikelihood = 0.0;
		int maxj = 0;
		int j = 1, k = 0;
		for (; k < NP && j < NP - 1; ++k) {
			double cl = (double)k / NP + 1.0e-6;
			for (; j < NP - 1 && (cl > cumlLikelihoods[j] || likelihoods[j] < 1.0e-7); ++j);
			state_prev.col(j).copyTo(state.col(k)); //pick
			if (likelihoods[j] > maxLikelihood) {
				maxLikelihood = likelihoods[j];
				maxj = j;
			}
		}
		for (; k < NP; ++k) state_prev.col(maxj).copyTo(state.col(k));
	}

	void update() {
		randomWalk();
	}

	void calcWeight(cv::Mat lidarDistImage) {
		evaluateEllipse(lidarDistImage);
	}

	cv::Vec2d boxMuller() {
		double x = (double)random() / random.max();
		double y = (double)random() / random.max();
		double z1 = sqrt(-2.0 * log(x)) * cos(2.0 * M_PI * y);
		double z2 = sqrt(-2.0 * log(x)) * sin(2.0 * M_PI * y);
		return cv::Vec2d(z1, z2);
	}

	void randomWalk() {
		const double WALK_WIDTH_1sigma = 1320.0 / 30.0 * 0.05 * 1.5;
		for (int p = 0; p < NP; ++p) {
			cv::Vec2d z = boxMuller();
			state(0, p) += z[0] * WALK_WIDTH_1sigma;
			state(1, p) += z[1] * WALK_WIDTH_1sigma;
			state(2, p) += boxMuller()[0] * 20;
			state(2, p) = fmod(state(2, p), 360);
		}
	}

	void evaluateEllipse(cv::Mat lidarDistImage) {
		for (int p = 0; p < NP; ++p) {
			// FIX: 使用 std::round 保持一致
			int t = ((int)std::round(state(2, p)) % 360 + 360) % 360;
			float ellipse_maxd = -1;
			int EvalCount = 0;

			// FIX: 使用 std::round 并添加边界检查
			cv::Point spos( (int)std::round(state(0,p)), (int)std::round(state(1,p)) );
			int lidarID = 0;
			for(size_t i = 0; i < lidarOccupancy.size(); ++i){
                // FIX: 增加边界检查，防止崩溃
                if (spos.x >= 0 && spos.x < lidarOccupancy[i].cols && spos.y >= 0 && spos.y < lidarOccupancy[i].rows) {
				    if(lidarOccupancy[i].at<uint8_t>(spos.y, spos.x) != 0){ // 注意OpenCV中坐标是(row, col)即(y, x)
					    lidarID = i; 
                        break;
				    }
                }
			}
			cv::Vec2d lspos = lidarPos[lidarID];

			for (int ci = 0; ci < TMPL_CNUM; ++ci) {
				double ellipse_cx = state(0, p) + ellipse_tmpl->contour[t][ci][0];
				double ellipse_cy = state(1, p) + ellipse_tmpl->contour[t][ci][1];
                // FIX: 使用 std::round
				int ellipse_icx = std::round(ellipse_cx);
				int ellipse_icy = std::round(ellipse_cy);

				cv::Vec2d ecp = cv::Vec2d(ellipse_cx, ellipse_cy) - lspos;
				double dp = ecp.ddot(ellipse_tmpl->norm[t][ci]);
				bool flg_visible = (dp < 0) &&
					ellipse_icx >= 0 && ellipse_icx < lidarDistImage.cols &&
					ellipse_icy >= 0 && ellipse_icy < lidarDistImage.rows;

				if (flg_visible) {
					float d = lidarDistImage.at<float>(ellipse_icy, ellipse_icx);
					ellipse_maxd = std::max(ellipse_maxd, d);
					++EvalCount;
				}
			}
			if (ellipse_maxd < 0) ellipse_maxd = 30000 * 0.05;

			// const double sigma = 5;
			const double sigma = 10;
			double e = exp(-ellipse_maxd * ellipse_maxd / sigma / sigma + EvalCount);

			const double EPS = 1.0e-7;
			likelihoods[p] = e < EPS ? 0 : e;
		}

		double sum_likelihood = 1.0e-5;
		for (int p = 0; p < NP; ++p) sum_likelihood += likelihoods[p];
		
        if(sum_likelihood > 1e-5) {
            for (int p = 0; p < NP; ++p) likelihoods[p] /= sum_likelihood;
        }

		cumlLikelihoods[0] = likelihoods[0];
		for (int p = 1; p < NP; ++p) {
			cumlLikelihoods[p] = likelihoods[p] + cumlLikelihoods[p - 1];
		}
	}

	void constructEllipseTmpl() {
		CV_Assert(ellipse_tmpl);

		const double axa = (double)ellipse_width / 2;
		const double axb = (double)ellipse_height / 2;
        #if _OPENMP
        #pragma omp parallel for
        #endif
		for (int d = 0; d < 360; ++d) {
			double dd = d * M_PI / 180.0;
			double ac = cos(dd);
			double as = sin(dd);
			cv::Matx22d rot(ac, -as, as, ac);
			for (int ci = 0; ci < TMPL_CNUM; ++ci) {
				double t = ci * 2.0 * M_PI / TMPL_CNUM;
				double x = axa * cos(t);
				double y = axb * sin(t);
				ellipse_tmpl->contour[d][ci] = rot * cv::Vec2d(x, y);
				cv::Vec2d n(axb * cos(t), axa * sin(t));
                double norm_val = cv::norm(n);
				if(norm_val > 1e-6) n /= norm_val;
				ellipse_tmpl->norm[d][ci] = rot * n;
			}
		}
	}
};

//複数追跡器管理クラス
class EllipseTracker2LSPool {
	std::vector<EllipseTracker2LS::Ptr> trackers_origin;
	std::deque<EllipseTracker2LS::Ptr> trackers_pool;
	int ID_Count = 0;

public:
	std::deque<EllipseTracker2LS::Ptr> trackers;

	/*!
	 * @brief Constructor
	 */
	EllipseTracker2LSPool(int ntrackers, int NP, int ellipse_width, int ellipse_height, const std::vector<cv::Vec2d> &lidarPos, const std::vector<cv::Mat> &lidarOccupancy) {
		CV_Assert(ntrackers > 0);
		trackers_origin.resize(ntrackers);
        // Pass a shared pointer for the template to subsequent trackers
        EllipseTracker2LS::EllipseTemplate::Ptr shared_template = nullptr;
		trackers_origin[0].reset(new EllipseTracker2LS(NP, ellipse_width, ellipse_height, lidarPos, lidarOccupancy, shared_template));
        shared_template = trackers_origin[0]->ellipse_tmpl;

		for (int i = 1; i < ntrackers; ++i) {
			trackers_origin[i].reset(new EllipseTracker2LS(NP, ellipse_width, ellipse_height, lidarPos, lidarOccupancy, shared_template));
		}
		trackers_pool = std::deque<EllipseTracker2LS::Ptr>(trackers_origin.begin(), trackers_origin.end());
	}
	/*!
	 * @brief Destructor
	 */
	~EllipseTracker2LSPool() {
	}

	/*!
	* @brief add tracker
	*/
	EllipseTracker2LS::Ptr add(int initX, int initY, int initAngl) {
		if (trackers_pool.empty()) {
			fprintf(stderr, "EllipseTracker2LS::add : No trackers remained in the pool\n");
			return nullptr;
		}

		EllipseTracker2LS::Ptr t = trackers_pool.front();
		trackers_pool.pop_front();
		t->init(initX, initY, initAngl);
		t->id = ID_Count++;
		trackers.push_back(t);

		return t;
	}

	/*!
	* @brief execute tracking for single frame
	*/
	void next(cv::Mat lidarDistImage) {
		for (size_t i = 0; i < trackers.size(); ++i)
			trackers[i]->next(lidarDistImage);
	}

	/*!
	* @brief remote tracker by ID
	*/
	bool removeByID(int id) {
		auto it = std::find_if(trackers.begin(), trackers.end(), [id](const EllipseTracker2LS::Ptr& t){ return t->id == id; });
        if(it != trackers.end()){
            trackers_pool.push_back(*it);
            trackers.erase(it);
            return true;
        } else {
            fprintf(stderr, "EllipseTracker2LS::removeByID : No such id of tracker\n");
			return false;
        }
	}

	/*!
	* @brief remote tracker by index of trackers array
	*/
	bool removeByOrder(int i) {
		if (!(i >= 0 && i < trackers.size())) {
			fprintf(stderr, "EllipseTracker2LS::removeOrder : Index of tracker out of range\n");
			return false;
		}
		EllipseTracker2LS::Ptr t = trackers[i];
		trackers.erase(trackers.begin() + i);
		trackers_pool.push_back(t);
		return true;
	}
	
	EllipseTracker2LS::Ptr operator[](int i) {
		return trackers[i];
	}
	
	size_t size() const {
		return trackers.size();
	}

	bool empty() const {
		return trackers.empty();
	}
};