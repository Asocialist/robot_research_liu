/*
 * 山田さんの作ったアカデミックパックのプログラムのラッパー
 * たった2行で車椅子が動き出す．スゴイ！
 * Author: suzuryo Date: 2015.5.26
 * Modified:
 *  2015.5.27 - Modify Read/Write method
 *            - Create function: stop
 *            - Create function: toCJmode
 *                *** jwvehicle_serial_protocol.cpp, jwvehicle_serial_protocol.cpp をCJコマンドに対応するように変更
 *            - Modify destructor: 終了時にCJコマンドを呼んでアカデミックモードからジョイスティックモードに変更するようにした
 *            - Modify move: 閾値を設定
 */
#ifndef __JWVEHICLE_WRAPPER_H__
#define __JWVEHICLE_WRAPPER_H__

#include<Windows.h>
#include<process.h>

#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#include "jwvehicle_serial.hpp"
#include "jwvehicle_serial_protocol.hpp"

namespace jwvehicle{
	class jwvehicle_wrapper{
		device_t dev;
		OVERLAPPED ovWaitComm;
		OVERLAPPED ovRead;
		OVERLAPPED ovWrite;

		//非同期処理用
		bool isLoop;
		//HANDLE thRWHandle;
		HANDLE thReadHandle;
		HANDLE thWriteHandle;
		HANDLE thChkReachedHandle;
		HANDLE thTimerHandle;
		HANDLE stopEvent;
		HANDLE writeEvent;
		HANDLE readEvent;
		HANDLE reachedEvent;
		HANDLE timerEvent;

		CRITICAL_SECTION wcs;
		char wlen;
		char wbuf[1024];
		CRITICAL_SECTION rcs;
		char rlen;
		char rbuf[1024];
		
		//コマンド送出間隔
		enum{CMD_SPAN=50};

	public:
		//コンストラクタ：インスタンス生成と同時にシリアル通信を開始し，アカデミックモードに移行する．
		//comNo: 開くCOMポート timeout: 接続処理のタイムアウト時間[ms]．この時間を過ぎるとプログラムを終了させる．
		jwvehicle_wrapper(char *comNo, unsigned int timeout=3000);
		//デストラクタ：シリアル通信を終了する．ジョイスティック操作モードに戻す．
		~jwvehicle_wrapper();

		//車椅子を動かす
		//-100~100(%)で割合指定
		//vel: 前進速度 turn: 回転速度(正:左回転)
		void move(int vel, int turn);
		//車椅子を止める
		void stop();
		
		//スレッド用コールバック関数＠ユーザは使わない
		friend static unsigned int __stdcall runThread_Read(void *t_this);
		friend static unsigned int __stdcall runThread_Write(void *t_this);
		friend static unsigned int __stdcall runThread_ChkReached(void *t_this);
		friend static unsigned int __stdcall runThread_Timer(void *t_this);

	private:
		//送信(送信バッファにコピーする)
		int write(char *buf);
		//受信(受信バッファからコピーする)
		int read(char *buf);
		//同期受信(受信バッファに書き込まれるまでブロッキング)
		//timeout: ブロッキングのタイムアウト時間の指定[ms]
		int readOnBlock(char *buf,int timeout=INFINITE);

		//アカデミックモードに移行する．アカデミックモードになるまでブロックする．
		void toCAmode();
		//ジョイスティックモードに移行する．ジョイスティックモードになるまでブロックする．
		void toCJmode();

		void startRead();
		void startWrite();
		void startChkReached();
	};

}
//別名
typedef jwvehicle::jwvehicle_wrapper JWVehicle;

#endif //__JWVEHICLE_WRAPPER_H__
