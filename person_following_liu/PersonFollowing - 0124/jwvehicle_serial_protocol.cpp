/*
 * jwvehicle_serial_protocol.cpp
 *
 *  Created on: 2014/08/05
 *      Author: tyamada
 */

#include "jwvehicle_serial_protocol.hpp"
#include <string.h>
#include <stdio.h>

namespace jwvehicle {
	static const char _Token_Header = '@' ;
	static const char _Token_Footer = 0x0a;

	static const char Format_CA[] = "@CA\r\n";
	static const char Format_CS[] = "@CS%+04d%+04d\r\n";
	static const char Format_CD[] = "@CD%+04d%+04d\r\n";
	static const char Format_CL[] = "@CL\r\n";
	static const char Format_CJ[] = "@CJ\r\n";

	static const char Token_CA[3] = { 'C', 'A', '\0' };
	static const char Token_CS[3] = { 'C', 'S', '\0' };
	static const char Token_CD[3] = { 'C', 'D', '\0' };
	static const char Token_CL[3] = { 'C', 'L', '\0' };
	static const char Token_CJ[3] = { 'C', 'J', '\0' };


	static const char Format_RA[] = "@RA%24s\r\n";
	static const char Format_RS[] = "@RS%3d%04d%04d%05d%05d\r\n";
	static const char Format_RD[] = "@RD%3d%04d%04d%05d%05d\r\n";
	static const char Format_RL[] = "@RL%3d%04d%04d\r\n";
	static const char Format_RJ[] = "@RJ%24s\r\n";

	static const char Token_RA[3] = { 'R', 'A', '\0' };
	static const char Token_RS[3] = { 'R', 'S', '\0' };
	static const char Token_RD[3] = { 'R', 'D', '\0' };
	static const char Token_RL[3] = { 'R', 'L', '\0' };
	static const char Token_RJ[3] = { 'R', 'J', '\0' };

	static const char Token_RE[2] = { 'E', '\0' };
}


int jwvehicle::cmd_createCA( char *dest ) {
	return sprintf(dest, Format_CA);
}

int jwvehicle::cmd_createCS( char *dest, int rate_trans, int rate_ang ) {
	return sprintf(dest, Format_CS, rate_trans, rate_ang);
}

int jwvehicle::cmd_createCD( char *dest, int rate_trans, int rate_ang ) {
	return sprintf(dest, Format_CD, rate_trans, rate_ang);
}

int jwvehicle::cmd_createCL( char *dest ) {
	return sprintf(dest, Format_CL);
}

int jwvehicle::cmd_createCJ( char *dest ) {
	return sprintf(dest, Format_CJ);
}



int jwvehicle::cmd_findheader( const char *src ) {
	int i = 0;
	for( i = 0; src[i] != '\0' && src[i] != _Token_Header; i++ ) ;
	return src[i] == _Token_Header ? i : -1;
}


int jwvehicle::cmd_findfooter( const char *src ) {
	int i = 0;
	for( i = 0; src[i] != '\0' && src[i] != _Token_Footer; i++ ) ;
	return src[i] == _Token_Footer ? i : -1;
}

int jwvehicle::cmd_pop( char *dest, char *src, int *len ) {
	int i = 0;
	int len_cmd = 0;

	if( (i = cmd_findfooter(src)) < 0  )	return -1;
	len_cmd = i + 1;

	// get cmd
	memcpy( dest, src, len_cmd );
	memset( dest + len_cmd, 0, 1);

	// pop cmd
	*len -= len_cmd;
	if( *len > 0 ) {
		memmove( src, src + len_cmd, *len );
	}
	memset(src + *len, 0, len_cmd);

	return len_cmd;
}

int jwvehicle::cmd_check( const char *cmd, int len ) {
	int ret  = -1;
	if( cmd[0] != _Token_Header ) return ret;

	if ( memcmp(cmd + 1, Token_RA, sizeof(Token_RA) - 1) == 0) {
		ret = CMD_RA;
	}
	else if ( memcmp(cmd + 1, Token_RS, sizeof(Token_RS) - 1) == 0 ) {
		ret = CMD_RS;
	}
	else if ( memcmp(cmd + 1, Token_RD, sizeof(Token_RD) - 1) == 0 ) {
		ret = CMD_RD;
	}
	else if ( memcmp(cmd + 1, Token_RL, sizeof(Token_RL) - 1) == 0 ) {
		ret = CMD_RL;
	}
	else if ( memcmp(cmd + 1, Token_RJ, sizeof(Token_RJ) - 1) == 0 ) {
		ret = CMD_RJ;
	}
	else if ( memcmp(cmd + 1, Token_RE, sizeof(Token_RE) - 1) == 0 ) {
		ret = CMD_RE;
	}
	else if ( memcmp(cmd + 1, Token_CA, sizeof(Token_CA) - 1) == 0 ) {
		ret = CMD_CA;
	}
	else if ( memcmp(cmd + 1, Token_CS, sizeof(Token_CS) - 1) == 0 ) {
		ret = CMD_CS;
	}
	else if ( memcmp(cmd + 1, Token_CD, sizeof(Token_CD) - 1) == 0 ) {
		ret = CMD_CD;
	}
	else if ( memcmp(cmd + 1, Token_CL, sizeof(Token_CL) - 1) == 0 ) {
		ret = CMD_CL;
	}
	else if ( memcmp(cmd + 1, Token_CJ, sizeof(Token_CJ) - 1) == 0 ) {
		ret = CMD_CJ;
	}
	else {
		ret = -1;
	}

	return ret;
}

int jwvehicle::cmd_parseRA(const char *cmd, char *version ) {
	char v[32];
	int nscan = 0;

	memset(v, 0, sizeof(v));
	nscan = sscanf(cmd, Format_RA, v);
	if( version ) strcpy(version, v);
	return nscan == 1 ? 0 : -1;
}

int jwvehicle::cmd_parseRS(const char *cmd, int *clock, int *rate_trans, int *rate_ang, int *rpm_right, int *rpm_left) {
	int c;
	int r_t, r_a;
	int r_r, r_l;
	int nscan;

	nscan = sscanf(cmd, Format_RS, &c, &r_t, &r_a, &r_r, &r_l);
	if( clock )			*clock = c;
	if( rate_trans )	*rate_trans = r_t;
	if( rate_ang )		*rate_ang = r_a;
	if( rpm_right ) 	*rpm_right = r_r;
	if( rpm_left )	 	*rpm_left = r_l;

	return nscan == 5 ? 0 : -1;
}

int jwvehicle::cmd_parseRD(const char *cmd, int *clock, int *rate_trans, int *rate_ang, int *rpm_right, int *rpm_left) {
	int c;
	int r_t, r_a;
	int r_r, r_l;
	int nscan;

	nscan = sscanf(cmd, Format_RD, &c, &r_t, &r_a, &r_r, &r_l);
	if( clock )			*clock = c;
	if( rate_trans )	*rate_trans = r_t;
	if( rate_ang )		*rate_ang = r_a;
	if( rpm_right ) 	*rpm_right = r_r;
	if( rpm_left )	 	*rpm_left = r_l;

	return nscan == 5 ? 0 : -1;
}

int jwvehicle::cmd_parseRJ(const char *cmd, char *version ) {
	char v[32];
	int nscan = 0;

	memset(v, 0, sizeof(v));
	nscan = sscanf(cmd, Format_RJ, v);
	if( version ) strcpy(version, v);
	return nscan == 1 ? 0 : -1;
}

int jwvehicle::cmd_parseRL( const char *cmd, int *clock, int *tilt_UD, int *tilt_LR ){
	int c;
	int t_UD, t_LR;
	int nscan;

	nscan = sscanf(cmd, Format_RL, &c, &t_UD, &t_LR);
	if( clock )			*clock = c;
	if( tilt_UD )		*tilt_UD = t_UD;
	if( tilt_LR )		*tilt_LR = t_LR;

	return nscan == 3 ? 0 : -1;
}

