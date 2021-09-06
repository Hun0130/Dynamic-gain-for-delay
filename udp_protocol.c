#include "udp_protocol.h"

void make_protocol(unsigned char *header, int len, unsigned int seq, int mode){
	header[0] = STX;
	header[1] = PLEN;
	header[2] = (unsigned char)len + HEADERLEN;
	header[3] = SEQ;
	header[4] = (unsigned char)seq / 256;
	header[5] = (unsigned char)seq % 256;

	if(mode == 1)
		header[6] = ACTUATOR;
	else if(mode == 2)
		header[6] = SENSOR;
	else{
		printf("make protocol mode error\n");
		exit(1);
	}
}
