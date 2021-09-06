#include "log_manager.h"
 
void Make_file_name(char* fhead, char* fname){	
	time_t t = time(NULL);
    struct tm tm = *localtime(&t);

	char log_head[256] = "log_";
	//char head[256] = "log_plant/plant_";
	char head[256];
	char body[256];
	char tail[5] = ".txt";
	
    sprintf(head,"%s%s/%s_",log_head, fhead, fhead);

    sprintf(body,"%d-%02d-%02d-%02d-%02d-%02d",tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	strncat(head,body,sizeof(body));
	strncat(head,tail,sizeof(tail));
	printf("debug: %s\n",head);
	strcpy(fname, head);
}
/*
int main(void){
	char file_head[256] = "plant";
	char file_name[256];
	//time_t t = time(NULL);
    //struct tm tm = *localtime(&t);
	int fd;

	Make_file_name("plant", file_name);
	
	if(0>(fd=open(file_name,O_WRONLY|O_CREAT|O_EXCL,0644)))
	{
		perror("File open error!");
		exit(0);
	}
	
	write(fd,file_name,strlen(file_name));
		  
    printf("%s\n",file_name);
	close(fd);
			 
	return 0;
 }
*/

