#include "matrix.h"
#include "udp_protocol.h"
#include "log_manager.h"
//void timer();
//int createTimer( timer_t *timerID, int sec, int msec );

struct State_Handler{
	struct Matrix* sysA;
	struct Matrix* sysB;
	struct Matrix* sysC;
	struct Matrix* x;
	double* u;
	double* y;
	
};


struct State_Handler plant;
double sim_time=0;
int sim_count=0;



//************************Main************************************************
int main(){

	//Physical plant setting
	double m_ball=0.11;
	double R_ball=0.015;
	double J_ball=9.99/1000000;
	double gravity=9.8;
	double H;
	H=m_ball*gravity/(J_ball/(R_ball*R_ball)+m_ball);
	printf("physical value H= %lf\n",H);
	double DC_gain=0.284369901518013;
	double ref_signal=0.5/DC_gain;
	H=7;

	/*
		Set physical system dynamics
	*/

	printf("Start controller main()!!\n");	
	int i;
	int iter=10;
	double A[DIMENSION][DIMENSION]={{0, 1, 0, 0},{0, 0, H, 0},{0, 0, 0, 1},{0, 0, 0, 0}};
	double B[DIMENSION][1]={{0}, {0}, {0}, {1}};
	double C[1][DIMENSION]={1, 0, 0, 0};
	double Kd[1][DIMENSION]={3.5165, 6.0935, 35.4275, 8.0612};							// Controller gain
	double Ld[DIMENSION][1]={{2.8654}, {11.8146}, {3.7094}, {3.2602}};					// Luenberger observer
	double u=0;	plant.u=&u;
	double y=0;	plant.y=&y;
	int current_time=0;
	double x_hat_0[DIMENSION][1]={{0}, {0}, {0}, {0}};									// Initial state estimation x_hat
	printf("Start matrix setting!!\n");
	struct Matrix sysA; 	sysA=Init_Mat(DIMENSION,DIMENSION,A);	plant.sysA=&sysA;
	struct Matrix sysB;   sysB=Init_Mat(DIMENSION,1,B);	plant.sysB=&sysB;
	struct Matrix sysC;   sysC=Init_Mat(1,DIMENSION,C);	plant.sysC=&sysC;
	struct Matrix sysAd;	sysAd=get_Transition_Matrix(sysA,iter,SAMPLING_PERIOD);
	struct Matrix sysBd;  sysBd=get_Bd(sysA, sysB, SAMPLING_PERIOD);
	struct Matrix controller_Kd;  controller_Kd=Init_Mat(1,DIMENSION,Kd);
	struct Matrix observer_Ld;    observer_Ld=Init_Mat(DIMENSION,1,Ld);
	
	//struct Matrix x;  x=Init_Mat(DIMENSION,1,x0);	plant.x=&x;
	struct Matrix x_hat;  x_hat=Init_Mat(DIMENSION,1,x_hat_0);
		
	//Make log file
	int fd;
	char file_name[256];
	char log_message[256];
	Make_file_name("controller",file_name);

	if(0>(fd=open(file_name,O_WRONLY|O_CREAT|O_EXCL,0644))){
		perror("File open error\n");
		exit(0);
	}
	
	/*
		Socket programming

	*/
		
	printf("Start socket setting\n");
	int sock;
	int server_addr_size, client_addr_size;
	struct sockaddr_in server_addr, client_addr;

	unsigned char buff_rcv[BUFFER_SIZE];			// Receive buffer
	unsigned char buff_snd[BUFFER_SIZE];			// Send buffer
	unsigned char Header[HEADERLEN];				// Header buffer
	int len=0;
	char u_value[50]={0,};							//Control input u(t) buffer
	char sensor_val[50]={0,};						//Sensor value y(t) buffer (from physical system)
	int u_len;
	unsigned int seq=0;
	
	sock = socket(PF_INET,SOCK_DGRAM,0);
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
		Just wait the sensor output y(t).
		When the sensor output y(t) is entered, While() loop calculates and sends the control input u(t).
	*/
	
	while(1){

		client_addr_size=sizeof(client_addr);
		
		len=recvfrom(sock, buff_rcv, BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_addr_size);	// Wait the control input signal.

		/*
			If the controller receive the kill signal from the physical system, controller must shut down the program. 
		*/

		if(!strncmp(buff_rcv,"kill",4)){
			printf("%s\n",buff_rcv);
			break;
		}
	
		// When the controller receive the packet,

		if(len>0){
			if(buff_rcv[HEADERLEN-1]==SENSOR){
				seq=(unsigned int)buff_rcv[4]*256+(unsigned int)buff_rcv[5]; 	// Get sequence of the sensor packet. (Redundant check)
				for(i=0; i<len-HEADERLEN; i++){
					sensor_val[i]=buff_rcv[HEADERLEN+i];						// Write the sensor output to sensor buffer.
				}
				y=atof(sensor_val);												// Update the sensor output y(t).
			}
			
			u=ref_signal+update_u(controller_Kd, &x_hat);						// Calculate the control input u(t).
			update_x_hat(sysAd, &x_hat, sysBd, u, observer_Ld, y, sysC);		// Update the state estimation x_hat(t). - Luenberger observer -.
			
			printf("%lf\t%lf\t%lf\t%lf\n",seq*SAMPLING_PERIOD, y, u, abs_double(y-x_hat.mat[0][0]));
			/*
				Write the log (Time, y(t), u(t), r(t) -> Residual)
			*/
			sprintf(log_message,"%lf\t%lf\t%lf\t%lf\n",seq*SAMPLING_PERIOD, y, u, abs_double(y-x_hat.mat[0][0]));
			write(fd,log_message,strlen(log_message));
			
			// Transform control input u(t) from (double) to char[]
			u_len=sprintf(u_value, "%.6lf",u);
			make_protocol(Header, u_len, seq, 1); seq++;	// Make protocol header.
			
			for(i=0; i<HEADERLEN; i++){						// Write header to send buffer.
				buff_snd[i]=Header[i];
			}
			for(i=0; i<u_len; i++){
				buff_snd[HEADERLEN+i]=u_value[i];			// Write u(t) to send buffer.
			}
			sendto(sock, buff_snd, HEADERLEN+u_len, 0, (struct sockaddr*)&client_addr, sizeof(client_addr)); // Send control input signal u(t).

		}
		len=0;
	}
	// Return the memory for control system.
	Matrix_free(sysA); Matrix_free(sysAd); Matrix_free(sysB);  Matrix_free(sysBd); Matrix_free(sysC); Matrix_free(controller_Kd); Matrix_free(observer_Ld); Matrix_free(x_hat);
	close(sock);
	close(fd);
	return 0;
}

