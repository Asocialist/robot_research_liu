#ifndef __AURGLIB_H__
#define __AURGLIB_H__


#include<Windows.h>
#include<process.h>

#include<stdio.h>

#include<Urg_driver.h>

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"Setupapi.lib")

#ifdef _DEBUG
#pragma comment(lib,"urg_cpp_d.lib")
#else
#pragma comment(lib,"urg_cpp.lib")
#endif

class aurglib{
	qrk::Urg_driver urg;
	std::vector<long> data_distance;
	std::vector<unsigned short> data_intensity;
	long stamp;
	qrk::Lidar::measurement_type_t measure_type;

	HANDLE hThread;
	CRITICAL_SECTION cs;
	bool loopFlg;

public:
	aurglib():loopFlg(false){
		InitializeCriticalSection(&cs);
	}
	~aurglib(){
		if(loopFlg) end();
		DeleteCriticalSection(&cs);
	}

	bool start(char *devname, qrk::Lidar::connection_type_t _type=qrk::Lidar::Ethernet, qrk::Lidar::measurement_type_t _measure_type=qrk::Urg_driver::Distance){
		if(loopFlg) return false;

		if(_type==qrk::Lidar::Ethernet){
			if(!urg.open(devname, qrk::Urg_driver::Default_port, qrk::Lidar::Ethernet)){
				fprintf(stderr,"Error on aurglib::start: %s\n",urg.what());
				return false;
			}
		}else if(_type==qrk::Lidar::Serial){
			if(!urg.open(devname, qrk::Urg_driver::Default_baudrate, qrk::Lidar::Serial)){
				fprintf(stderr,"Error on aurglib::start: %s\n",urg.what());
				return false;
			}
		}else{
			return false;
		}

		measure_type=_measure_type;
		if(!urg.start_measurement(measure_type,-1,0)){
			fprintf(stderr,"Error on aurglib::start: %s\n",urg.what());
			return false;
		}

		loopFlg=true;
		hThread=(HANDLE)_beginthreadex(NULL,0,__aurglib_runThread,this,0,NULL);

		return true;

	}
	bool end(){
		if(!loopFlg) return false;

		loopFlg=false;
		if(WaitForSingleObject(hThread,500)==WAIT_TIMEOUT)
			TerminateThread(hThread,0);
		return true;
	}

	bool getDist(int distData[1080]){
		if(!loopFlg) return false;

		std::vector<long> _data_distance;

		EnterCriticalSection(&cs);
		_data_distance=data_distance;
		LeaveCriticalSection(&cs);

		for(int i=0;i<1080 && i<_data_distance.size();i++){
			distData[i]=_data_distance[i];
		}

		return true;
	}
	bool getDistIntensity(int distData[1080], int intensityData[1080]){
		if(!loopFlg || measure_type!=qrk::Lidar::Distance_intensity) return false;

		std::vector<long> _data_distance;
		std::vector<unsigned short> _data_intensity;

		EnterCriticalSection(&cs);
		_data_distance=data_distance;
		_data_intensity=data_intensity;
		LeaveCriticalSection(&cs);

		for(int i=0;i<1080 && i<_data_distance.size();i++){
			distData[i]=_data_distance[i];
		}
		for(int i=0;i<1080 && i<_data_intensity.size();i++){
			intensityData[i]=_data_intensity[i];
		}

		return true;
	}


	static friend unsigned int __stdcall __aurglib_runThread(void *_param){
		((aurglib*)_param)->run();
		return 0;
	}

private:
	void run(){
		long _stamp;
		std::vector<long> _data_distance;
		std::vector<unsigned short> _data_intensity;

		while(loopFlg){
			if(measure_type==qrk::Lidar::Distance){
				if(!urg.get_distance(_data_distance,&_stamp)){
					fprintf(stderr,"Error on aurglib::run: %s\n",urg.what());
					loopFlg=false;
					break;
				}
			}
			else if(measure_type==qrk::Lidar::Distance_intensity){
				if(!urg.get_distance_intensity(_data_distance,_data_intensity,&_stamp)){
					fprintf(stderr,"Error on aurglib::run: %s\n",urg.what());
					loopFlg=false;
					break;
				}
			}

			EnterCriticalSection(&cs);
			switch(measure_type){
			case qrk::Lidar::Distance_intensity:
				data_intensity=_data_intensity;
			case qrk::Lidar::Distance:
				data_distance=_data_distance;
				break;
			}
			stamp=_stamp;
			LeaveCriticalSection(&cs);
		}

	}
};

#endif //__AURGLIB_H__
