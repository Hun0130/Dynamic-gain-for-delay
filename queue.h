// circular queue for checking delay
#include <stdio.h>                                
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
# define MAX_QUEUE_SIZE 5

typedef struct QueueType{
	double element_data[MAX_QUEUE_SIZE];
	int front, rear;
}QueueType;

// Queue Init
void init_queue(QueueType* q);

// Check Queue is empty
int is_empty(QueueType* q);

//Check Queue is full
int is_full(QueueType* q); 

// Check if the queue is full and insert.
void enqueue(QueueType* q, double data);

// Check if the queue is empty and delete it.
double dequeue(QueueType* q);

double sum_queue(QueueType *q);