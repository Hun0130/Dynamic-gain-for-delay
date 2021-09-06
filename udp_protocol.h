#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>


#define BUFFER_SIZE 1024
#define STX 68
#define PLEN 69
#define SEQ 70
#define HEADERLEN 7
#define SENSOR 83
#define ACTUATOR 65

//Protocol//
// STX|LEN|packet_length|SEQ|sequence_high|sequence_low|SENSOR or ACTUATOR| +payload (sensor value)

void make_protocol(unsigned char *header, int len, unsigned int seq, int mode);

