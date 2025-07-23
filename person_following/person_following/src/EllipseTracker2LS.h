/*!
 * @file EllipseTracker2LS.h
 * @brief 楕円追跡パーティクルフィルタ（Lidar2つ使う版）
 * @details
 * 	2つ使う時はどっちの観測領域に追跡対象が居るかでセンサ位置の評価が変わる．
 *  体がどう見えているかの判定もそれに適応させないといけない（楕円モデルと観測がズレるとマズい）．
 * @author Ryota Suzuki
 * @date 2025/6/16
	\copyright Copyright 2024 Ryota Suzuki. All rights reserved.
	\license This project is released under the MIT License.
*/

#pragma once

#define _USE_MATH_DEFINES
#include<math.h>

#include<vector>
#include<memory>
#include<algorithm>
#include<random>

#include<opencv2/opencv.hpp>

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
	std::vector<cv::Mat1b> lidarOccupancy;

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
	 * @param[in] NP Number of particles
	 * @param[in] ellipse_width Width of ellipse
	 * @param[in] ellipse_height Height of ellipse
	 * @param[in] lidarPos Position of LiDAR on image
	 * @param[in] ellipse_tmpl Memory for ellipse template. If null, new one will be created. You should share this to others for efficiency.
	 * @detail Constructor
	 */
	EllipseTracker2LS(int NP, int ellipse_width, int ellipse_height, const std::vector<cv::Vec2d> &lidarPos, const std::vector<cv::Mat1b> &lidarOccupancy, EllipseTemplate::Ptr ellipse_tmpl = nullptr)
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

		//TODO: feedable seed
		random.seed(5489);
	}
	/*!
	 * @brief Destructor
	 * @detail Destructor
	 */
	~EllipseTracker2LS() {
	}

	/*!
	 * @brief Initialize particles
	 * @param[in] initX initial posX
	 * @param[in] initY initial posY
	 * @param[in] initAngl initial angle
	 * @return false if failed
	 * @detail Initialize particles
	 */
	bool init(int initX, int initY, int initAngl) {
		cv::Mat1d ip(3, 1, std::array<double, 3>{(double)initX, (double)initY, (double)initAngl}.data());
		//cv::Mat1d ip(3, 1); ip << (double)initPos.x, (double)initPos.y, 0.0;
#if _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < NP; ++i) ip.copyTo(state.col(i));
		//state.row(0).setTo(initPos.x); state.row(1).setTo(initPos.y); state.row(2).setTo(0.0);
		state.copyTo(state_prev);
		std::fill(likelihoods.begin(), likelihoods.end(), 1.0 / NP);
		cumlLikelihoods[0] = likelihoods[0];
		for (int i = 1; i < NP; ++i) cumlLikelihoods[i] = likelihoods[i] + cumlLikelihoods[i - 1];

		flg_exp_calced = false;
		return true;
	}

	/*!
	 * @brief Get final calculated pose as expectation
	 * @return result pose
	 * @detail Get final calculated pose as expectation
	 */
	cv::Vec3d getPos() {
		//calc exp
		if (!flg_exp_calced) {
			expPos = cv::Vec3d(0, 0, 0);
			cv::Vec2d av(0, 0);
			for (int i = 0; i < NP; ++i) {
				expPos[0] += likelihoods[i] * state(0, i);
				expPos[1] += likelihoods[i] * state(1, i);
				//expPos[2] += likelihoods[i] * state(2, i); //向きの角度値の平均はあやしい
				double t = state(2, i) * M_PI / 180.0;
				av += likelihoods[i] * cv::Vec2d(cos(t), sin(t)); //ベクトル和で向きの平均を取る

			}
			expPos[2] = atan2(av[1], av[0]) * 180 / M_PI;
			flg_exp_calced = true;
		}
		return expPos;
	}

	/*!
	 * @brief execute tracking for single frame
	 * @param[in] lidarImage distance image of lidar image as observation (must be CV_32FC1)
	 * @return false if failed
	 * @detail execute tracking for single frame
	 */
	bool next(cv::Mat lidarDistImage) {
		CV_Assert(lidarDistImage.channels() == 1 && lidarDistImage.depth() == CV_32F);
		flg_exp_calced = false;
		//リサンプリング
		resample();
		//状態遷移
		update();
		//重み計算
		calcWeight(lidarDistImage);
		return true;
	}

private:
	//リサンプリング
	void resample() {
		//累積尤度を等間隔サンプリング
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
		//端数処理
		for (; k < NP; ++k) state_prev.col(maxj).copyTo(state.col(k));
	}

	//伝播
	void update() {
		//とりあえずランダムウォーク
		randomWalk();
	}

	//各サンプルの重み計算
	void calcWeight(cv::Mat lidarDistImage) {
		evaluateEllipse(lidarDistImage);
	}

	//-----------------------------------------------------------------

	//正規乱数生成（N(0,1)）
	cv::Vec2d boxMuller() {
		double x = (double)random() / random.max(); // [0,1]
		double y = (double)random() / random.max(); // [0,1]
		double z1 = sqrt(-2.0 * log(x)) * cos(2.0 * M_PI * y); // N(0,1)
		double z2 = sqrt(-2.0 * log(x)) * sin(2.0 * M_PI * y); // N(0,1)
		return cv::Vec2d(z1, z2);
	}

	//ランダムウォーク
	void randomWalk() {
		//1sigma:68% (95%, 99.7%, ...)
		const double WALK_WIDTH_1sigma = 1320.0 / 30.0 * 0.05 * 1.5; //1320[mm/s] / 30[fps] * 0.05[px/mm]
		for (int p = 0; p < NP; ++p) {
			cv::Vec2d z = boxMuller();
			state(0, p) += z[0] * WALK_WIDTH_1sigma;
			state(1, p) += z[1] * WALK_WIDTH_1sigma;
			//state(2, 0) = (double)random() * 360 / ((unsigned long long)random.max()+1); //[0, 360)
			//state(2, p) += (double)random() * 90 / ((unsigned long long)random.max() + 1) - 45; //[-45, 45)
			state(2, p) += boxMuller()[0] * 20;
			state(2, p) = fmod(state(2, p), 360); //-360~360に抑制
		}
	}

	//各サンプルを楕円評価
	void evaluateEllipse(cv::Mat lidarDistImage) {
		//TODO:計算最適化

		//その位置に楕円テンプレートを置く
		//その楕円の，観測までの最大距離＝評価値
//#if _OPENMP
//#pragma omp parallel for
//#endif
		for (int p = 0; p < NP; ++p) {
			int t = ((int)::round(state(2, p)) % 360 + 360) % 360;
			float ellipse_maxd = -1;

			int ellipse_picx = -100;
			int ellipse_picy = -100;
			int EvalCount = 0;

			//どのLiDARの観測範囲にいるか判定
			cv::Point spos((int)::round(state(1,p)),(int)::round(state(0,p)));
			int lidarID=0;
			for(int i=0;i<lidarOccupancy.size();++i){
				if(lidarOccupancy[i].at<uint8_t>(spos)!=0){
					lidarID=i; break;
				}
			}
			cv::Vec2d lspos = lidarPos[lidarID];

			for (int ci = 0; ci < TMPL_CNUM; ++ci) {
				double ellipse_cx = state(0, p) + ellipse_tmpl->contour[t][ci][0];
				double ellipse_cy = state(1, p) + ellipse_tmpl->contour[t][ci][1];
				int ellipse_icx = ::round(ellipse_cx);
				int ellipse_icy = ::round(ellipse_cy);
				//重複マッチング排除
				if (ellipse_picx == ellipse_icx && ellipse_picy == ellipse_icy) continue;
				ellipse_picx = ellipse_icx; ellipse_picy = ellipse_icy;

				//見える？＝法線ベクトルがLiDARの方を向いている？　＆画像の範囲内？
				cv::Vec2d ecp = cv::Vec2d(ellipse_cx, ellipse_cy) - lspos;
				double dp = ecp.ddot(ellipse_tmpl->norm[t][ci]);
				bool flg_visible = (dp < 0) &&
					ellipse_icx >= 0 && ellipse_icx < lidarDistImage.cols &&
					ellipse_icy >= 0 && ellipse_icy < lidarDistImage.rows;

				if (flg_visible) {
					float d = lidarDistImage.at<float>((int)::round(ellipse_cy), (int)::round(ellipse_cx));
					ellipse_maxd = std::max(ellipse_maxd, d);
					++EvalCount;
				}
			}
			if (ellipse_maxd < 0) ellipse_maxd = 30000 * 0.05; //no hit => 30m
			else if (ellipse_maxd == 0) ellipse_maxd = 0;

			////固定値sigmaだけ
			//const double sigma = 20.0;
			//double e = exp(-ellipse_maxd * ellipse_maxd / sigma / sigma);

			//exp(EvalCount)を掛けて原作再現
			const double sigma = 5;
			double e = exp(-ellipse_maxd * ellipse_maxd / sigma / sigma + EvalCount);

			const double EPS = 1.0e-7;
			likelihoods[p] = e < EPS ? 0 : e;
		}

		//並列化のためにatomic演算は外出し
		double sum_likelihood = 1.0e-5;
		for (int p = 0; p < NP; ++p) sum_likelihood += likelihoods[p];
		//std::reduce(likelihoods.begin(), likelihoods.end(), 0.0); //c++17
		//{
		//	cv::Mat1d l(1, NP, (double*)likelihoods.data());
		//	cv::Mat1d _l;
		//	cv::reduce(l, _l, 1, cv::REDUCE_SUM);
		//	sum_likelihood = l(0, 0);
		//}
		likelihoods[0] /= sum_likelihood;

		cumlLikelihoods[0] = likelihoods[0];
		for (int p = 1; p < NP; ++p) {
			likelihoods[p] /= sum_likelihood;
			cumlLikelihoods[p] = likelihoods[p] + cumlLikelihoods[p - 1];
		}
	}

	//楕円テンプレート作成
	void constructEllipseTmpl() {
		CV_Assert(ellipse_tmpl);

		const double axa = (double)ellipse_width / 2;
		const double axb = (double)ellipse_height / 2;
#if _OPENMP
#pragma omp parallel for
#endif //_OPENMP
		for (int d = 0; d < 360; ++d) {
			//arg angl of ellipse
			double dd = d * M_PI / 180.0;
			double ac = cos(dd);
			double as = sin(dd);
			cv::Matx<double, 2, 2> rot(std::array<double, 4>{ac, -as, as, ac}.data()); //rowmajor data order
			//sampling contour of ellipse
			for (int ci = 0; ci < TMPL_CNUM; ++ci) {
				double t = ci * 2.0 * M_PI / TMPL_CNUM;
				double x = axa * cos(t);
				double y = axb * sin(t);
				//rorate by arg angl
				ellipse_tmpl->contour[d][ci] = rot * cv::Vec2d(x, y);

				/*
				* dx/dt = -a * sin(t)
				* dy/dt =  b * cos(t)
				* n = (dy/dt, -dx/dt) (right-hand)
				*/
				cv::Vec2d n(axb * cos(t), axa * sin(t)); //normal vector
				n /= sqrt(n.ddot(n)); //normalize
				ellipse_tmpl->norm[d][ci] = rot * n; //rorate by arg angl
			}
		}
	}
};

//複数追跡器管理クラス
class EllipseTracker2LSPool {
	//メモリソース
	std::vector<EllipseTracker2LS::Ptr> trackers_origin;
	//空いている追跡器キュー
	std::deque<EllipseTracker2LS::Ptr> trackers_pool;

	//ID累計
	int ID_Count = 0;

public:
	//現行追跡器
	std::deque<EllipseTracker2LS::Ptr> trackers;

	/*!
	 * @brief Constructor
	 * @param ntrackers Number of trackers to manage
	 * @param NP Number of particles
	 * @param ellipse_width Width of template ellipse
	 * @param ellipse_width Height of template ellipse
	 * @detail Constructor
	 */
	EllipseTracker2LSPool(int ntrackers, int NP, int ellipse_width, int ellipse_height, const std::vector<cv::Vec2d> &lidarPos, const std::vector<cv::Mat1b> &lidarOccupancy) {
		CV_Assert(ntrackers > 0);
		trackers_origin.resize(ntrackers);
		trackers_origin[0].reset(new EllipseTracker2LS(NP, ellipse_width, ellipse_height, lidarPos, lidarOccupancy, nullptr));
		//trackers_origin[0]->id = ID_Count++;
		for (int i = 1; i < ntrackers; ++i) {
			trackers_origin[i].reset(new EllipseTracker2LS(NP, ellipse_width, ellipse_height, lidarPos, lidarOccupancy, trackers_origin[0]->ellipse_tmpl));
			//trackers_origin[i]->id = ID_Count++;
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
	* @param initX initial posX
	* @param initY initial posY
	* @param initAngl initial angle
	* @return Smart pointer to the added tracker
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
	* @param lidarDistImage distance image of lidarImage (must be CV_32FC1)
	*/
	void next(cv::Mat lidarDistImage) {
		for (int i = 0; i < trackers.size(); ++i)
			trackers[i]->next(lidarDistImage);
	}

	/*!
	* @brief remote tracker by ID
	* @param id ID of tracker
	*/
	bool removeByID(int id) {
		int i = -1;
		for (int _i = 0; _i < trackers.size(); ++_i) {
			if (trackers[_i]->id == id) {
				i = _i;
				break;
			}
		}
		if (i >= 0) {
			EllipseTracker2LS::Ptr t = trackers[i];
			trackers.erase(trackers.begin() + i);
			trackers_pool.push_back(t);
			return true;
		}
		else {
			fprintf(stderr, "EllipseTracker2LS::removeByID : No such id of tracker\n");
			return false;
		}
	}
	/*!
	* @brief remote tracker by index of trackers array
	* @param i index of trackers array
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

	////
	//Overload functions of vector

	/*!
	* @brief accessor to a tracker
	* @param i index of trackers array
	*/
	EllipseTracker2LS::Ptr operator[](int i) {
		return trackers[i];
	}
	/*!
	* @brief number of trackers
	* @return number of trackers
	*/
	size_t size() {
		return trackers.size();
	}
	/*!
	* @brief check if trackers are empty
	* @return true if no trackers
	*/
	bool empty() {
		return trackers.empty();
	}
};
