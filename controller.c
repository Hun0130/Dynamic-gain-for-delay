#include "matrix.h"
#include "udp_protocol.h"
#include "log_manager.h"

// micro second measurement
#include <time.h>
#include <sys/time.h>
#define GET_TIME(t) gettimeofday(t, NULL);
#define ELAPS_TIME(e, s) (e.tv_sec + e.tv_usec/1000000.0) - (s.tv_sec + s.tv_usec/1000000.0)
// save micro second value
struct timeval past; struct timeval now;
struct timeval time_val;

// u = -Kx + U_c
#define UC 0

// structure for state equation x'(t)= Ax(t) + Bu(t), y(t) = Cx(t) + Du(t) (D = 0)
struct State_Handler{
	// matrix for A
	struct Matrix* sysA;
	// matrix for B
	struct Matrix* sysB;
	// matrix for C
	struct Matrix* sysC;
	// plant state vector x(t)
	struct Matrix* x;
	// contorl input u(t)
	double* u;
	// sensor output y(t)
	double* y;
};

struct State_Handler plant;
double sim_time = 0;
int sim_count = 0;
double delay = 0;

// Main fuction
int main(){

	// ================== Controller setting ===================

	printf("Start controller\n");	
	int iter = 10;

	// Controller gain
	double K1 = 0.0659;
	double K2 = -0.0583;
	double Kd[1][DIMENSION] = {K1, K2};

	double u = 0;	plant.u = &u;
	double y = 0;	plant.y = &y;
	// value of state x(t)
	double x1 = 0;  double x2 = 0;
	int current_time = 0;
	double A[DIMENSION][DIMENSION]={{0, 1}, {0, -41.5769}};
	double B[DIMENSION][1]={{0}, {384.615}};
	double C[1][DIMENSION]={1, 0};
	struct Matrix sysA; 	sysA = Init_Mat(DIMENSION, DIMENSION, A);	plant.sysA = &sysA;
	struct Matrix sysB;   sysB = Init_Mat(DIMENSION, 1, B);	plant.sysB = &sysB;
	struct Matrix sysC;   sysC = Init_Mat(1, DIMENSION, C);	plant.sysC = &sysC;
	struct Matrix sysAd;	sysAd = get_Transition_Matrix(sysA,iter, SAMPLING_PERIOD);
	struct Matrix sysBd;  sysBd = get_Bd(sysA, sysB, SAMPLING_PERIOD);
	struct Matrix controller_Kd;  controller_Kd=Init_Mat(1,DIMENSION, Kd);
	
	// Make log file
	int fd;
	char file_name[256];
	char log_message[256];
	Make_file_name("controller",file_name);

	if(0 > (fd = open(file_name,O_WRONLY|O_CREAT|O_EXCL,0644))){
		perror("File open error\n");
		exit(0);
	}
	
	// Socket programming		
	int sock;
	int server_addr_size, client_addr_size;
	struct sockaddr_in server_addr, client_addr;

	// Receive buffer
	unsigned char buff_rcv[BUFFER_SIZE];		
	// Send buffer	
	unsigned char buff_snd[BUFFER_SIZE];
	// Header buffer			
	unsigned char Header[HEADERLEN];				
	int len = 0;
	// Control input u(t) buffer
	char u_value[50] = {0,};			
	int u_len;
	// State1 buffer (from plant)
	char state1_val[50] = {0,};
	// State2 buffer (from plant)			
	char state2_val[50] = {0,};		
	
	unsigned int seq = 0;
	
	sock = socket(PF_INET,SOCK_DGRAM,0);
	// Socket error
	if(sock == -1){
		printf("socket creation error\n");
		exit(1);
	}
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
	
	/*
		Control input signal calculation.
		Just wait the state x(t)
		When the state x(t) is entered, While() loop calculates and sends the control input u(t).
	*/

	while(1){
		client_addr_size = sizeof(client_addr);
		// Wait the control input signal.
		len = recvfrom(sock, buff_rcv, BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_addr_size);	
		// Measure time
		GET_TIME(&time_val);
		// update past time
		past = now;
		// Save now time
		now = time_val;
		// If the controller receive the kill signal from the physical system, controller must shut down the program. 
		if(!strncmp(buff_rcv,"kill", 4)){
			printf("%s\n",buff_rcv);
			break;
		}

		// When the controller receive the packet,
		if(len > 0){
			int check = 0;
			int len1 = 0;
			if(buff_rcv[HEADERLEN-1] == SENSOR){
				// Get sequence of the sensor packet. (Redundant check)
				seq = (unsigned int)buff_rcv[4] * 256 + (unsigned int)buff_rcv[5]; 	
				for(int i = 0; i < len - HEADERLEN; i++){
					if (buff_rcv[HEADERLEN+i] == '-'){
						check = 1;
						continue;
					}
					if (check == 0){
						// Write the state1 value to state1 buffer
						state1_val[i] = buff_rcv[HEADERLEN + i];
						len1 = len1 + 1;
					}
					if (check == 1){
						// Write the state2 value to state2 buffuer
						state2_val[i - len1 - 1] = buff_rcv[HEADERLEN + i];
					}
				}
				x1 = atof(state1_val);
				x2 = atof(state2_val);									
			}
			// Calculate the control input u(t)
			u = -K1 * x1 - K2 * x2 + UC;
			
			printf("time: %lf\t u(t): %lf\t delay(us): %f\n", sim_time * SAMPLING_PERIOD, u, ELAPS_TIME(now, past));
			// Write the log (Time, y(t), u(t), r(t) -> Residual)
			// If delay is more than 10s, do not log
			if (ELAPS_TIME(now, past) < 10.0){
				sprintf(log_message,"%lf\t%lf\t%f\n", (sim_time - 1) * SAMPLING_PERIOD, u, ELAPS_TIME(now, past));
				write(fd,log_message, strlen(log_message));
			}
			
			// Transform control input u(t) from (double) to char[]
			u_len = sprintf(u_value, "%.6lf", u);
			// Make protocol header.
			make_protocol(Header, u_len, seq, 1); seq++;	
			sim_time = sim_time + 1.0;
			
			// Write header to send buffer.
			for(int i = 0; i < HEADERLEN; i++){						
				buff_snd[i] = Header[i];
			}

			// Write u(t) to send buffer.
			for(int i = 0; i < u_len; i++){
				buff_snd[HEADERLEN+i] = u_value[i];			
			}
			// Send control input signal u(t).
			sendto(sock, buff_snd, HEADERLEN+u_len, 0, (struct sockaddr*)&client_addr, sizeof(client_addr)); 
		}
		len = 0;
	}
	// Return the memory for control system.
	Matrix_free(sysA); Matrix_free(sysAd); Matrix_free(sysB);  Matrix_free(sysBd); Matrix_free(sysC); Matrix_free(controller_Kd);
	close(sock);
	close(fd);
	return 0;
}

