#include "queue.h"

// Queue Init
void init_queue(QueueType* q) {
	q->front = q->rear = 0;
}

// Check Queue is empty
int is_empty(QueueType* q) {
	return (q->front == q->rear);
}

//Check Queue is full
int is_full(QueueType* q) {
	return (q->front == ((q->rear+1)%MAX_QUEUE_SIZE));
}

// Check if the queue is full and insert.
void enqueue(QueueType* q, double data) {
	if (is_full(q)) {
        dequeue(q);
	}
    q->rear = (q->rear + 1) % MAX_QUEUE_SIZE;
    q->element_data[q->rear] = data;
}

// Check if the queue is empty and delete it.
double dequeue(QueueType* q) {
	if (is_empty(q)) {
		exit(1);
	}
	else {
		q->front = (q->front + 1) % MAX_QUEUE_SIZE;
		double data = q->element_data[q->front];
		return data;
	}
}

double sum_queue(QueueType *q) {
    if (is_empty(q)) {
		return 0.0;
	}
	else {
		double return_val = 0.0;
		if (!is_empty(q)) {
			int i = q->front;
			do {
				i = (i + 1) % MAX_QUEUE_SIZE;
				return_val += q->element_data[i];
				if (i == q->rear)
					break;
			} while (i != q->front);
		}
        return return_val;
	}
}