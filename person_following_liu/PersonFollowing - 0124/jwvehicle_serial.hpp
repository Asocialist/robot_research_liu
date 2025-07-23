/**
 * \file /include/jwvehicle/jwvehicle_serial.hpp
 **/
#ifndef JWVEHICLE_SERIAL_HPP_
#define JWVEHICLE_SERIAL_HPP_


#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

// ---> type declaration
namespace jwvehicle {
#if defined(_WIN32)
	typedef HANDLE			device_t;
#elif defined(__linux__)
	typedef int				device_t;
#endif
}
// <--- type declaration


// ---> function declaration
namespace jwvehicle {
	int open( device_t *dev, const char *port );
	int close( device_t *dev  );
	int is_open(device_t *dev);

	int read( device_t dev, char *dest, unsigned int size, OVERLAPPED *o);
	int write( device_t dev, const char *src, unsigned int size, OVERLAPPED *o);
}
// <--- function declaration

#endif // JWVEHICLE_SERIAL_HPP_
