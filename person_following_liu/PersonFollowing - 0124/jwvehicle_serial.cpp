/**
 * @file /jwvehicle_controller/src/lib/jwvehicle_controller.cpp
 *
 * @brief File comment
 **/

// visual studio secure function
#if defined(_MSC_VER) // visual studio
#pragma warning(disable:4996)
#else
#endif

#include "jwvehicle_serial.hpp"

#include <windows.h>

#include <string.h>

int jwvehicle::open( device_t *dev, const char *port ) {
	DCB 			dcb;
	COMMTIMEOUTS 	timeout;

	*dev = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

	if( *dev == INVALID_HANDLE_VALUE ) {
		return -1;
	}

	// set communication state
	if( !GetCommState(*dev, &dcb) ) {
		return -1;
	}
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = CBR_38400;
	dcb.Parity = EVENPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;
	dcb.EvtChar='\n';
	if( !SetCommState(*dev, &dcb) ) {
		jwvehicle::close(dev);
		return -1;
	}


	// set blocking
	timeout.ReadIntervalTimeout = 0;
	timeout.ReadTotalTimeoutMultiplier = 0;
	timeout.ReadTotalTimeoutConstant = 1;
	timeout.WriteTotalTimeoutMultiplier = 3;
	timeout.WriteTotalTimeoutConstant = 2;
	if( !SetCommTimeouts(*dev, &timeout) ) {
		jwvehicle::close(dev);
		return -1;
	}

	return 0;
}


int jwvehicle::close( device_t *dev ) {
	if( *dev == INVALID_HANDLE_VALUE ) {
		return -1;
	}

	CloseHandle(*dev);
	*dev = INVALID_HANDLE_VALUE;
	return 0;
}


int jwvehicle::is_open( device_t *dev ) {
	return *dev != INVALID_HANDLE_VALUE;
}



int jwvehicle::read( device_t dev, char *dest, unsigned int size, OVERLAPPED *o)
{
	DWORD nread;
	memset(dest, 0, size);
	if( !ReadFile(dev, dest, (DWORD)size, &nread, o) ) return -1;
	return (int)nread;
}


int jwvehicle::write( device_t dev, const char *src, unsigned int size, OVERLAPPED *o)
{
	DWORD nwrite = 0;

	if( !WriteFile(dev, src, size, &nwrite, o) )	return -1;
	if( !FlushFileBuffers(dev) ) 					return -1;

	return nwrite;
}
