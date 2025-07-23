/*
*    URG キャプチャ　ヘッダファイル
*
*/
#ifndef __MYURGCAPTURE_H__
#define __MYURGCAPTURE_H__

int  init_URG(char *ComNo);
void start_URG(void);
void GetDistanceData(int URG_Distance[]);
int  close_URG(void);

#endif //__MYURGCAPTURE_H__
