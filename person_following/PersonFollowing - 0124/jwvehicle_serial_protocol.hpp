/*
 * jwvehicle_serial_protocol.hpp
 *
 *  Created on: 2014/08/05
 *      Author: tyamada
 */

#ifndef JWVEHICLE_SERIAL_PROTOCOL_HPP_
#define JWVEHICLE_SERIAL_PROTOCOL_HPP_

// ---> const definition
namespace jwvehicle {
	const int  CMD_CA = 0;
	const int  CMD_CS = 1;
	const int  CMD_CD= 2;
	const int  CMD_CL= 3;
	const int  CMD_CJ= 14;

	const int  CMD_RA = 4;
	const int  CMD_RS = 5;
	const int  CMD_RD = 6;
	const int  CMD_RL = 7;
	const int  CMD_RJ = 18;

	const int  CMD_RE = 8;
}
// ---> const definition

// ---> function declaration
namespace jwvehicle {
	/**
	 * \brief create CA command string
	 * \param [out] *dest   : command
	 * \note dest must have size of greater than or equal 6bytes
	 * \return 6: length of command string
	 * \return <0 : fail
	 */
	int cmd_createCA( char *dest );
	/**
	 * \brief create CS command string
	 * \param [out] *dest       : command
	 * \param [in]   rate_trans : translational velocity order [%]
	 * \param [in]   rate_ang   : angular velocity order [%]
	 * \note dest must have size of greater than or equal 13bytes
	 * \return 13: length of command string
	 * \return <0 : fail
	 */
	int cmd_createCS( char *dest, int rate_trans, int rate_ang );
	/**
	 * \brief create CS command string
	 * \param [out] *dest       : command
	 * \param [in]   rate_trans : translational velocity order [%]
	 * \param [in]   rate_ang   : angular velocity order [%]
	 * \note dest must have size of greater than or equal 13bytes
	 * \return 13: length of command string
	 * \return <0 : fail
	 */
	int cmd_createCD( char *dest, int rate_trans, int rate_ang );
	/**
	 * \brief create CS command string
	 * \param [out] *dest       : command
	 * \note dest must have size of greater than or equal 13bytes
	 * \return 5: length of command string
	 * \return <0 : fail
	 */
	int cmd_createCL( char *dest );
	/**
	 * \brief create CJ command string
	 * \param [out] *dest   : command
	 * \note dest must have size of greater than or equal 6bytes
	 * \return 6: length of command string
	 * \return <0 : fail
	 */
	int cmd_createCJ( char *dest );


	/**
	 * \brief find command header('@')
	 * \param [in]  *src 		: read data from jw-vehicle control device (called academic pack)
	 * \return >=0 : index of header
	 * \return <0  : not found header
	 */
	int cmd_findheader(const char *src);

	/**
	 * \brief find command footer(Line Feed)
	 * \param [in]  *src 		: read data from jw-vehicle control device (called academic pack)
	 * \return >=0 : index of footer
	 * \return <0  : not found footer
	 */
	int cmd_findfooter(const char *src);

	/**
	 * \brief sprite command string
	 * \param [out]    *dest		: single command
	 * \param [in]     *src 		: read data from jw-vehicle control device (called academic pack)
	 * \param [in/out]  len			: length of read data(src)
	 * \return length of single command string
	 */
	int cmd_pop( char *dest, char *src,  int *len );

	/**
	 * \brief parse and analyze kind of receive command
	 * \param [in]  *src 		: single command
	 * \return kind of receive command
	 * \return <0 : fail
	 */
	int cmd_check( const char *cmd, int len );
	/**
	 * \brief parse RA command string
	 * \param [in]  *cmd 		: received RA command
	 * \param [out] *version 	: jw-vehicle control device version (required greater than 24byte)
	 * \return 0  : success
	 * \return <0 : fail
	 */
	int cmd_parseRA( const char *cmd, char *version );
	/**
	 * \brief parse RS command string
	 * \param [in]  *cmd 		: received RS command
	 * \param [out] *clock		: clock [10ms]
	 * \param [out] *rate_trans : translational velocity order [%]
	 * \param [out] *rate_ang	: angular velocity order [%]
	 * \param [out] *rpm_right 	: right wheel velocity [rpm]
	 * \param [out] *rpm_left	: left wheel velocity [rpm]
	 * \return 0  : success
	 * \return <0 : fail
	 */
	int cmd_parseRS( const char *cmd, int *clock, int *rate_trans, int *rate_ang, int *rpm_right, int *rpm_left );
	/**
	 * \brief parse RD command string
	 * \param [in]  *cmd 		: received RS command
	 * \param [out] *clock		: clock [10ms]
	 * \param [out] *rate_trans : translational velocity order [%]
	 * \param [out] *rate_ang	: angular velocity order [%]
	 * \param [out] *rpm_right 	: right wheel velocity [rpm]
	 * \param [out] *rpm_left	: left wheel velocity [rpm]
	 * \return 0  : success
	 * \return <0 : fail
	 */
	int cmd_parseRD( const char *cmd, int *clock, int *rate_trans, int *rate_ang, int *rpm_right, int *rpm_left );
	/**
	 * \brief parse RJ command string
	 * \param [in]  *cmd 		: received RJ command
	 * \param [out] *version 	: jw-vehicle control device version (required greater than 24byte)
	 * \return 0  : success
	 * \return <0 : fail
	 */
	int cmd_parseRJ( const char *cmd, char *version );
	/**
	 * \brief parse RL command string
	 * \param [in]  *cmd 		: received RS command
	 * \param [out] *clock		: clock [10ms]
	 * \param [out] *tilt_fb 	: joystick tilt rate in front-back direction[%]
	 * \param [out] *tilt_side	: joystick tilt rate in side direction [%]
	 * \return 0  : success
	 * \return <0 : fail
	 */
	int cmd_parseRL( const char *cmd, int *clock, int *tilt_UD, int *tilt_LR );
}
// <--- function declaration

#endif /* JWVEHICLE_SERIAL_PROTOCOL_HPP_ */
