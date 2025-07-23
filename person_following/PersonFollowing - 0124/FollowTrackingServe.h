#pragma once


#include <string>
#include "json11.hpp"
#include <winsock2.h>


class FollowTrackingServer {
public:
	FollowTrackingServer(std::string pathConfigFile);
	~FollowTrackingServer();

	std::string GetURGPort();
	int SendTracking(double x, double y, double k);

private:
	FollowTrackingServer();
	std::string StringSendFormat(double x, double y, double k);


	SOCKET sock;
	struct sockaddr_in addr;


	std::string UDP_address = "127.0.0.1";
	int UDP_port = 50100;
	std::string URG_port = "COM4";

	double offset_pos_x = 0.0;
	double offset_pos_y = 0.0;
	double offset_pos_t = 0.0;
	bool offset_reverse = false;
	double time_interval_min = 0.0;

	
};