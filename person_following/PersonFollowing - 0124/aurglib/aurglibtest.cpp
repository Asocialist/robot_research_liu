
#include<stdio.h>
#include<stdlib.h>

#define _USE_MATH_DEFINES
#include<math.h>

#include"aurglib.h"

int main(){
	aurglib urg;

	if(!urg.start("COM1",qrk::Lidar::Serial,qrk::Lidar::Distance_intensity)){
		fprintf(stderr,"Error on Urg_driver::open\n");
		exit(-1);
	}

	//char buf[1024];
	//sprintf(buf,"\"C:\\Program Files (x86)\\gnuplot\\bin\\pgnuplot.exe\"");
	//puts(buf);
	//FILE *pp=_popen(buf,"w");
	//if(!pp){
	//	perror("pgnuplot");
	//	exit(-1);
	//}
	//Sleep(1000);

	//fprintf(pp,"set xrange [-5000:5000]\n");
	//fprintf(pp,"set yrange [-5000:5000]\n");
	//fflush(pp);

	int distData[1080];
	int intensityData[1080];

	for(int i=0;i<5000;++i){
		urg.getDistIntensity(distData,intensityData);


		//**********************************************
		// Write your code here
		//**********************************************


		//** sample **

		//FILE *fp=fopen("tmp.txt","w");
		//for(int d=0;d<1080;++d){
		//	//printf("%d,",data_l[d]);
		//	double rad=d*1.5*M_PI/1080.0;
		//	double x=distData[d]*cos(rad);
		//	double y=distData[d]*sin(rad);
		//	fprintf(fp,"%g %g\n",x,y);
		//}

		//fflush(fp);
		//fclose(fp);

		//fprintf(pp,"plot \"tmp.txt\" w p\n");
		//fflush(pp);

		Sleep(30);
	}

	//fclose(pp);

	urg.end();


	return 0;
}


