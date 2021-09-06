//plant.c//
#include "matrix.h"
#include "udp_protocol.h"
#include "log_manager.h"
#include <time.h>
#include <signal.h>

// port number of server
#define SERVER_PORT 4000
// period (s)
#define SEC 0
// period (ms)
#define MSEC 1

// Declare
void timer();
int createTimer( timer_t *timerID, int sec, int msec );

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
	// time of disturbance
	double* disturbance_time;
	// value of disturbance
	double* disturbance_value;
	// Integration of absolute error
	double* IAE;
	// Reference signal
	double* ref_signal;
	
};

struct State_Handler plant; 

// structure for network environment setting
struct Environment_Setting{
	// simulation end
	double* Sim_end;
	// server address 
	struct sockaddr_in* server_addr;
	// client address
	struct sockaddr_in* client_addr;
	// socket pointer
	int* sock;
	// size of server address
	int* server_addr_size;
	// size of client address
	int* client_addr_size;
};

// simulation time (ms)
double sim_time = 0;
// simulation counter
int sim_count = 1;
// sequecence of packet
unsigned int packet_seq = 0;

// socket programming
struct sockaddr_in server_addr, client_addr;	
int sock, server_addr_size, client_addr_size;

// File descriptor to make log file
int fd;					

// timer var, second, ms
int createTimer(timer_t *timerID, int sec, int msec)  
{  
    struct sigevent         te;  
    struct itimerspec       its;  
    struct sigaction        sa;  
    int                     sigNo = SIGRTMIN;  

    /* Set up signal handler. */  
    sa.sa_flags = SA_SIGINFO;  
	// function timer() will be called
    sa.sa_sigaction = timer;     
    sigemptyset(&sa.sa_mask);  

    if (sigaction(sigNo, &sa, NULL) == -1)  
    {  
        printf("sigaction error\n");
        return -1;  
    }  

    /* Set and enable alarm */  
    te.sigev_notify = SIGEV_SIGNAL;  
    te.sigev_signo = sigNo;  
    te.sigev_value.sival_ptr = timerID;  
    timer_create(CLOCK_REALTIME, &te, timerID);  

    its.it_interval.tv_sec = sec;
    its.it_interval.tv_nsec = msec * 1000000;  
    its.it_value.tv_sec = sec;
    
    its.it_value.tv_nsec = msec * 1000000;
    timer_settime(*timerID, 0, &its, NULL);  
    return 0;  
}

/*	Timer for real-time physical process.
	This function is operated only Linux OS because this code use Linux system functions.
	This function is called for every period (can set with createTimer()) 
	Now, period is 1ms(MSEC) */
void timer()
{
	// origin: unsigned char sensor_value[50]={0,};	(Sensor value buffer)
	// State1 value buffer
	char state1_buffer[50] = {0,};
	// State2 value buffer
	char state2_buffer[50] = {0,};
	// Tx buffer (will be transmitted) (BUFFER_SIZE : 1024)
	unsigned char Tx[BUFFER_SIZE];
	memset(Tx, 0, sizeof(Tx));
	// Header creation buffer(HEADERLEN: 7)
	unsigned char Header[HEADERLEN];
	// length of whole packet (state1 + '-' + state2)
	int len; 
	// length of state1_buffer
	int len1; 
	// length of state2_buffer
	int len2;
	char log_message[256];
	// SAMPLING_PERIOD (s) should be unsigned int format.
	// Now It's 2s
	int sensing_period = (int) (SAMPLING_PERIOD * 1000);
	
	// ========================== Disturbance Code ===========================================
	if(sim_time > *(plant.disturbance_time) && sim_time < *(plant.disturbance_time) + SAMPLING_PERIOD){ 
		// Inflict the disturbance during SAMPLING_PERIOD
		*(plant.u) += *(plant.disturbance_value);	
	}
	// ========================== Disturbance Code ===========================================

	// ========================== Physical system State Update Code ==========================
	// Time update (1ms).
	printf("t: %lf\t", sim_time); 
	sim_time = sim_time + (MSEC * 0.001);	
	sim_count = sim_count + 1;				

	// Print the current state x(t), y(t), and u(t)
	printf("x(t): ");
	mat_print(mat_transpose(*(plant.x)));
	printf("y(t): %lf\t u(t): %lf\n", *(plant.y), *(plant.u));		

	// Update the phyiscla state x(t).
	update_state(*(plant.sysA), plant.x, *(plant.sysB), *(plant.u));	
	// Update the Senor output y(t).
	*(plant.y) = update_y(*(plant.sysC), plant.x);
	// Calculate the IAE.							
	*(plant.IAE) += abs_double(*(plant.ref_signal)-*(plant.y)) * 0.001;						
	// ========================== Physical system State Update Code ==========================

	// Sampling and transmiting the sensor output y(t) to controller. (Every 2s)
	if((sim_count % sensing_period) == 0){
		// origin : len = sprintf(sensor_value, "%lf", *(plant.y)); (Transform sensor output from (double) to char[].)
		// Transform state1 output from (double) to char[].
		len1 = sprintf(state1_buffer, "%lf", plant.x->mat[0][0]);
		// Transform state2 output from (double) to char[])
		len2 = sprintf(state2_buffer, "%lf", plant.x->mat[1][0]);
		len = len1 + len2 + 1;
		// Make header.											
		make_protocol(Header, len, packet_seq ,2);													
		
		// Fill the transmission buffer Tx[] with header and state x_0 and x_1.	
		for(int i = 0; i < HEADERLEN; i++)
			Tx[i] = Header[i];
		for(int i = 0; i < len; i++){
			if(i < len1){
				Tx[HEADERLEN + i] = state1_buffer[i];
			}
			if(i == len1){
				Tx[HEADERLEN + i] = '-'; 
			}
			if(i > len1){
				Tx[HEADERLEN + i] = state2_buffer[i - len1 - 1];
			}
		}
		// Send the state output to controller. (state1-state2)
		sendto(sock, Tx, len + HEADERLEN, 0, (struct sockaddr*) & server_addr, sizeof(server_addr));
		packet_seq ++;
	}
	// Log file writing
	sprintf(log_message, "%lf\t%lf\t%lf\n", sim_time, *(plant.y), *(plant.u));
	write(fd,log_message, strlen(log_message));
}


// Main fuction
int main(int argc, char* argv[]){
	printf("Start Plant\n");
	// ===================================== Plant Setting =======================================
	// Matrix Setting
	// A = [ 0  1        ]  B  = [ 0 ]   C = [ 1 0 ]
	//     [ 0  -41.5769 ]     = [ 384.615 ]
	// K = [ 0.75, 0.023]
	// t_s = 0.1s (100ms)
	double A[DIMENSION][DIMENSION]={{0, 1}, {0, -41.5769}};
	double B[DIMENSION][1]={{0}, {384.615}};
	double C[1][DIMENSION]={1, 1};

	struct Matrix sysA;   sysA = Init_Mat(DIMENSION,DIMENSION,A);	plant.sysA = &sysA;
	struct Matrix sysB;   sysB = Init_Mat(DIMENSION,1,B);	        plant.sysB = &sysB;
	struct Matrix sysC;   sysC = Init_Mat(1,DIMENSION,C);	        plant.sysC = &sysC;
	//struct Matrix sysAd;	sysAd=get_Transition_Matrix(sysA,iter,SAMPLING_PERIOD);
	//struct Matrix sysBd;  sysBd=get_Bd(sysA, sysB, SAMPLING_PERIOD);

	// Initial control input = 0 
	double u = 0 ;	plant.u = &u;		
	// Initial sensor output = 0 
	double y = 0;	plant.y = &y;

	// Simulation end at 30s
	double sim_end = 30;	
	// Current time 
	int current_time = 0;

	/*	Disturbance Code:
		If you want to inflict physical disturbance to plant, you can set up two variables
		disturbance_time: Disturbance start time;
		disturbance_value: Amplitude of disturbance [Not disturbance-> set 0]; 	*/

	double disturbance_time = 0; 	    plant.disturbance_time = &disturbance_time; 	
	double disturbance_value = 0;		plant.disturbance_value = &disturbance_value;

	// Initial state of the physical system.
	// x0 = [ 1 ]
	//	  = [ 1 ]
	double x0[DIMENSION][1]={{1}, {1}};	
	struct Matrix x;  x = Init_Mat(DIMENSION,1,x0);	plant.x = &x;
	
	// Set Integration of absolute error
	double IAE = 0;		plant.IAE=&IAE;			
	// Set Reference signal					
	double ref = 0.5;	plant.ref_signal = &ref;										
	// ===================================== Plant Setting =======================================

	// ===================================== Network Setting =====================================
	// Receive buffer : Store packets received from the controller
	char buff_rcv[BUFFER_SIZE];
	// buffer for control input signal u(t) in packet
	char actuator_val[BUFFER_SIZE]={0,};
	// When simulation finish, plant sends end_char to controller. Then, the controller process should be finished. 
	char end_char[5] = "kill";			

	// UDP socket programming(Domain, Type, Protocol)
	// Domain : PF_INET (IPv4)
	// SOCK_DGRAM : UDP,  SOCK_STREAM  : TCP,  SOCK_RAW : User_Defined
	// protoco : 0 (auto), IPPROTO_TCP, IPPROTO_UDP
	sock = socket(PF_INET, SOCK_DGRAM, 0);

	// socket return -1 : error
	if(sock == -1){
		printf("socket creation error\n");
		exit(1);
	}

	// Reference : https://jjoreg.tistory.com/entry/%EC%97%B4%ED%98%88%EA%B0%95%EC%9D%98-TCPIP 
	memset(&server_addr, 0, sizeof(server_addr));
	// Address system : AF_INET : IPv4 Internet protocol
	server_addr.sin_family = AF_INET;
	// 16bit PORT NUM = Controller UDP port = 4000
	// htons() : host byte order -> network byte order,  ntohs : network byte order -> host byte order
	server_addr.sin_port = htons(SERVER_PORT);	
	// controller 32bit IPv4 address : 127.0.0.1
	server_addr.sin_addr.s_addr	= inet_addr("127.0.0.1");
	// ===================================== Network Setting =====================================
	
	// ===================================== Logging Setting =====================================
	char file_name[256];
	Make_file_name("plant", file_name);
	
	if( 0 > (fd = open(file_name,O_WRONLY|O_CREAT|O_EXCL, 0644))){
		perror("Log File open error\n");
		exit(0);
	}
	// ===================================== Logging Setting =====================================

	// ===================================== Start Physical Process ==============================
	timer_t timerID;
	// Modify SEC and MSEC to change plant update period (now It's 1ms)
	createTimer(&timerID, SEC, MSEC);

	int message_len = 0;
	// Operate until the end of simulation
	while(sim_time < sim_end){ 
		// Wait the enter of control input u(t).
		message_len = recvfrom(sock, buff_rcv, BUFFER_SIZE, 0, (struct sockaddr*) & client_addr, &client_addr_size);
		// If entered packet is actuator packet.
		if(message_len > 0 || buff_rcv[HEADERLEN - 1] == ACTUATOR){	
			for(int i = 0; i < message_len - HEADERLEN; i++){
				// Parse the control input signal u(t) from the packet. 
				actuator_val[i] = buff_rcv[HEADERLEN+i];			
			}
			// Update the control input u(t).
			u = atof(actuator_val);	
		}
	}
	// Send kill signal to controller.
	sendto(sock, end_char, strlen(end_char)+1, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)); 
	// Close the socket.
	close(sock);

	// Return the memory for physical system
	Matrix_free(sysA); Matrix_free(sysB);  Matrix_free(sysC);  Matrix_free(x);
	printf("Integral of the Absolute Error (IAE)= %lf \n", IAE);

	// Log file close.
	close(fd);	
	return 0;
}