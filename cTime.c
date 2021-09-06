#include "cTime.h"

int createTimer(timer_t *timerID, int sec, int msec){
	struct sigevent         te;
    struct itimerspec       its;
    struct sigaction        sa;
    int                     sigNo=SIGRTMIN;

    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timer;
    sigemptyset(&sa.sa_mask);

    if (sigaction(sigNo, &sa, NULL)==-1){
	    printf("sigaction error\n");
        return -1;
    }

    te.sigev_notify = SIGEV_SIGNAL;
    te.sigev_signo=sigNo;
    te.sigev_value.sival_ptr= timerID;
    timer_create(CLOCK_REALTIME, &te, timerID);
    its. it_interval.tv_sec=sec;
    its. it_interval.tv_nsec=msec*1000000;
    its. it_value.tv_sec=sec;
    its. it_value.tv_nsec=msec*1000000;
    timer_settime(*timerID, 0, &its, NULL);
    
	return 0;
}

