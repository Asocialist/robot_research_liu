#include "FollowTrackingServe.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdlib.h>
#include <memory.h>
#include <stdio.h>
#include <cmath>




json11::Json LoadJsonFile(const std::string filename);
int main_test(int argc, char **argv) {
	//std::unique_ptr<FollowTrackingServer> ftServer;
	FollowTrackingServer* ftServer = nullptr;

	if (argc >= 2) {
		//ftServer = std::make_unique<FollowTrackingServer>()
		ftServer = new FollowTrackingServer(argv[1]);
	}
	else {
		std::cerr << "invalid argument." << std::endl;
		exit(1);
	}

	for (int i = 0; i < 300; i++) {
		ftServer->SendTracking(i, i/2.0, 1);
	}

	return 0;
}


FollowTrackingServer::FollowTrackingServer(std::string pathConfigFile){
	auto configure = LoadJsonFile(pathConfigFile);
	if (configure == nullptr) {
		std::cerr << "cannnoot read configure file." << std::endl;
		return;
	}

	UDP_port = configure["UDP_port"].is_null() ? UDP_port : configure["UDP_port"].int_value();
	UDP_address = configure["UDP_address"].is_null() ? UDP_address : configure["UDP_address"].string_value();
	URG_port = configure["URG_port"].is_null() ? URG_port : configure["URG_port"].string_value();
	offset_pos_x = configure["offset_pos_x"].is_null() ? offset_pos_x : configure["offset_pos_x"].number_value();
	offset_pos_y = configure["offset_pos_y"].is_null() ? offset_pos_y : configure["offset_pos_y"].number_value();
	offset_pos_t = configure["offset_pos_t"].is_null() ? offset_pos_t : configure["offset_pos_t"].number_value();
	offset_reverse = configure["offset_reverse"].is_null() ? offset_reverse : configure["offset_reverse"].bool_value();
	time_interval_min = configure["time_interval_min"].is_null() ? time_interval_min : configure["time_interval_min"].number_value();

	WSAData wsaData;
	WSAStartup(MAKEWORD(2, 0), &wsaData);
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_port);
	addr.sin_addr.S_un.S_addr = inet_addr(UDP_address.c_str());
	return;

}

FollowTrackingServer::~FollowTrackingServer(){
	closesocket(sock);
	WSACleanup();
}

std::string FollowTrackingServer::GetURGPort(){
	return URG_port;
}

int FollowTrackingServer::SendTracking(double x, double y, double k){
	std::string buf = StringSendFormat(x, y, k);
	sendto(sock, buf.c_str(), buf.size(), 0, (struct sockaddr *)&addr, sizeof(addr));
	return 0;
}

std::string FollowTrackingServer::StringSendFormat(double x, double y, double k)
{
	std::ostringstream buf;
	buf << "{ \"offset_pos\":{\"x\":" << offset_pos_x << ",\"y\":" << offset_pos_y << ",\"angle\":" << offset_pos_t << "},\"tracked_pos\":{\"x\":" << x << ",\"y\":" << y << "},\"k\":" << k <<"}";
	return buf.str();
}

json11::Json LoadJsonFile(const std::string filename) {

	std::ifstream ifs(filename);
	if (ifs.fail()) {
		std::cerr << "file not exist" << std::endl;
		return nullptr;
	}

	std::string buf((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	ifs.close();
	std::string err;
	auto jsonObj = json11::Json::parse(buf, err);
	std::cerr << err << std::endl;

	return jsonObj;
}
