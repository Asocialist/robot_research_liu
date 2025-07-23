/*
*　  URG キャプチャ
*    マルチスレッドに設定してコンパイル
*/
#include <windows.h>
#include <process.h>
#include <stdio.h>

#include "MyUrgCapture.h"

HANDLE     g_commHandle;			// 通信ポートハンドル
OVERLAPPED g_ReadOverlapped;		// リード用オーバラップ構造体
OVERLAPPED g_WriteOverlapped;		// ライト用オーバラップ構造体
BYTE rxBuffTmp[8192],rxBuff[4096];	// 通信バッファ  BYTE rxBuffTmp[4096],rxBuff[2048];

// スレッドのハンドル
unsigned __stdcall ReadThread(void *lpx);
static HANDLE g_hThread;
static DWORD  g_hThreadID;

CRITICAL_SECTION g_csReadThread;
volatile BOOL g_endFlag;			// 受信完了フラグ
volatile BOOL g_stopFlag;			// スレッド停止フラグ
volatile WORD g_totalCnt;			// 終端コード(line feed 連続2回)が入るまでの受信データ数(line feed とsum 除く)
volatile WORD g_distance[1080];		// 距離格納配列

void CommClose(void);
void CommOpen(char *ComNo);

////////////////////////////////////
// データ受信スレッド
////////////////////////////////////
unsigned __stdcall ReadThread(void *lpx)
{
	DWORD   Event, NumberOfBytes, Error;
	COMSTAT Comstat;
	BYTE    lfCnt = 0;

	EnterCriticalSection(&g_csReadThread);
	g_totalCnt = 0;
	LeaveCriticalSection(&g_csReadThread);
	while(1)
	{
		Sleep(10);

		// スレッド停止
		int stopFlag = 0;
		EnterCriticalSection(&g_csReadThread);
		stopFlag = g_stopFlag;
		LeaveCriticalSection(&g_csReadThread);
		if(stopFlag) break;

		WaitCommEvent(g_commHandle,&Event,&g_ReadOverlapped);	// 受信イベントの確認

		if(Event & EV_RXCHAR)									// 受信イベントが発生した場合
		{
			ClearCommError(g_commHandle,&Error,&Comstat);		// 受信バイト数の取得
			if(Comstat.cbInQue)
			{	// 受信データサイズ分バッファに読み込み
				if(!ReadFile(g_commHandle,rxBuffTmp,Comstat.cbInQue,&NumberOfBytes,&g_ReadOverlapped)){	// 動作未完了の場合
					if(GetLastError() == ERROR_IO_PENDING)	// 問題無しの場合
						GetOverlappedResult(g_commHandle,&g_ReadOverlapped,&NumberOfBytes,TRUE);	// 動作完了まで待機
				}

				for(unsigned int i = 23;i < Comstat.cbInQue;i++){ // i=23からデータ（sensorからのエコーバック、タイムスタンプ等とばしてデータ部分から処理）
					if(rxBuffTmp[i] == 0x0a){	// lfを検知した場合
						lfCnt++;				// rxBuffに lf はコピーしない
/**/					g_totalCnt--;			// lf の前に sum があるので sum を次ブロックの先頭データで上書きするために g_totalCnt を sum の場所に戻す
					}else{
						EnterCriticalSection(&g_csReadThread);
						rxBuff[g_totalCnt++] = rxBuffTmp[i];
						LeaveCriticalSection(&g_csReadThread);
						lfCnt = 0;
					}

					if(lfCnt == 2){				// lfを2回連続検知した場合
						lfCnt = 0;
/**/					g_totalCnt++;			//戻しすぎた分進める
						EnterCriticalSection(&g_csReadThread);
						g_endFlag = 1;			// 受信完了フラグを立てる
						LeaveCriticalSection(&g_csReadThread);
					}
				}
			}
		}
	}

	return 0;
}

void start_URG(){ // BMコマンドで距離測定開始
	int i;

	DWORD NumberOfByte;
	BYTE bmCommand[] = "BM";
	BYTE txBuff[64];
	BYTE *txBuffWork = txBuff;

	for(i=0;i<2;i++) *txBuffWork++ = bmCommand[i];
	*txBuffWork++ = 0xa;

	WriteFile(g_commHandle,txBuff,txBuffWork - txBuff,&NumberOfByte,&g_WriteOverlapped);

	return;
}

//////////////////////////////////////////
// URG 距離データの取得
//////////////////////////////////////////
void GetDistanceData(int URG_Distance[])
{
	DWORD NumberOfByte;
	int i,j,k;	

	BYTE gCommand[] = "GD0000108000";			// Gコマンド文字列 "G00076800";
	BYTE txBuff[64];						// 送信バッファ    BYTE txBuff[64];
	BYTE *txBuffWork = txBuff;

	for(i = 0;i < 12;i++) *txBuffWork++ = gCommand[i];//	for(i = 0;i < 9;i++) *txBuffWork++ = gCommand[i];
	*txBuffWork++ = 0xa;					// 終端文字(line feed)

	// 送信
	EnterCriticalSection(&g_csReadThread);
	g_endFlag = 0;
	LeaveCriticalSection(&g_csReadThread);
	WriteFile(g_commHandle,txBuff,txBuffWork - txBuff,&NumberOfByte,&g_WriteOverlapped);
	for(;;){
		int endFlag = 0;
		EnterCriticalSection(&g_csReadThread);
		endFlag = g_endFlag;
		LeaveCriticalSection(&g_csReadThread);
		if(endFlag) break;
		Sleep(1);
	}
	
	EnterCriticalSection(&g_csReadThread);
	j = 0;
	for(i=0;i<g_totalCnt;i+=3){ // ３キャラクタ(バイト)で１データ
		g_distance[j++] = ((rxBuff[i] - 0x30) << 12) + ((rxBuff[i+1] - 0x30) << 6) + (rxBuff[i+2] - 0x30); // デコードしながら距離情報を格納
	}
	g_totalCnt = 0;
	LeaveCriticalSection(&g_csReadThread);

	EnterCriticalSection(&g_csReadThread);
	for(k=0;k<1080;k++){
		URG_Distance[k] = (int)g_distance[k];
	}
	LeaveCriticalSection(&g_csReadThread);

	return;
}

//////////////////////////////////////////
// URG の開始
//////////////////////////////////////////
int init_URG(char *ComNo){
	// オーバラップ構造体の初期化
	ZeroMemory(&g_ReadOverlapped,  sizeof(OVERLAPPED));
	ZeroMemory(&g_WriteOverlapped, sizeof(OVERLAPPED));
	g_ReadOverlapped.hEvent  = CreateEvent(NULL, TRUE, FALSE, NULL);
	g_WriteOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	// 通信ポートのオープン
	CommOpen(ComNo);

	// スレッドの開始
	g_stopFlag = FALSE;	
	InitializeCriticalSection(&g_csReadThread);
	g_hThread = (HANDLE)_beginthreadex(NULL, 0, ReadThread, NULL, 0, (unsigned int *)&g_hThreadID);

	return 1;
}

//////////////////////////////////////////
// URG の終了
//////////////////////////////////////////
int close_URG(void)
{
	// スレッドの終了
	EnterCriticalSection(&g_csReadThread);
	g_stopFlag = TRUE;						// スレッド停止フラグを立てる
	LeaveCriticalSection(&g_csReadThread);
	WaitForSingleObject(g_hThread,2000);	// スレッドの終了を待つ

	// 通信ポートのクローズ
	CommClose();

	return 1;
}

////////////////////////////////////
// Commポートを開く
////////////////////////////////////
void CommOpen(char *ComNo)
{
	DCB	 dcb;
	BOOL retVal;

	// シリアルポートのオープン 使用するUSBポートに合わせて番号を指定する
	g_commHandle = CreateFile(ComNo,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_FLAG_OVERLAPPED,NULL);

	// 通信の詳細設定
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = CBR_115200;
	dcb.fBinary = TRUE;
	dcb.fParity = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fErrorChar = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fAbortOnError = FALSE;
	dcb.fDummy2 = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 0;
	dcb.XoffLim = 0;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.XonChar = 0;
	dcb.XoffChar = 0;
	dcb.ErrorChar = 0;
	dcb.EofChar = 0;
	dcb.EvtChar = 0;
	dcb.wReserved1 =0;

	retVal = SetCommState(g_commHandle,&dcb);		// DCB構造体の内容をシリアルポートに設定
	retVal = SetCommMask(g_commHandle,EV_RXCHAR);	// イベントの設定（受信イベント）
	retVal = SetupComm(g_commHandle,0x1000,0x1000);	// 入出力バッファのサイズ設定
}

////////////////////////////////////
// Commポートを閉じる
////////////////////////////////////
void CommClose(void)
{
	CloseHandle(g_commHandle);
}
