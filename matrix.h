#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

// dimension of the state
#define DIMENSION 2
#define NUM_INPUT 1
#define NUM_OUTPUT 1
#define SAMPLING_PERIOD 0.1
#define EPSILON 0.000001
#define DELTA 0.001

//Sampling time [s]

double abs_double(double n);

int factorial(int n);

struct Matrix {   // 구조체 정의
    double **mat;        // 구조체 멤버 1
    unsigned int num_row;              // 구조체 멤버 2
    unsigned int num_col;    // 구조체 멤버 3
};

struct Kalman_filter{
	struct Matrix* P_temp;
	struct Matrix* A;
	struct Matrix* C;
	struct Matrix* P;
	struct Matrix* I; 
	struct Matrix* Kalman_gain;
	struct Matrix* x_kalman_temp;
	struct Matrix* x_kalman;
	double* cov_v;
	double* cov_w;
	double* y;
};

struct Matrix Init_Mat(int num_row, int num_col, double A[][num_col]);

struct Matrix Init_Mat_dptr(int num_row, int num_col, double** A);

struct Matrix mat_mul(struct Matrix A, struct Matrix B);

struct Matrix mat_mul_scalar(struct Matrix A, double c);

struct Matrix mat_transpose(struct Matrix A);

void mat_cpy(struct Matrix *A, struct Matrix *B);

void mat_print(struct Matrix A);

void Matrix_free(struct Matrix A);

double mat_trace(struct Matrix A);

double mat_det(struct Matrix A);

struct Matrix mat_inv(struct Matrix A);

struct Matrix get_Transition_Matrix(struct Matrix A, unsigned int iter, double dtime);

struct Matrix get_Bd(struct Matrix A, struct Matrix B, double dtime);

void update_state(struct Matrix A, struct Matrix *x, struct Matrix B, double u);

double update_y(struct Matrix C, struct Matrix *x);

double update_u(struct Matrix K, struct Matrix *x_hat);

void update_x_hat(struct Matrix Ad, struct Matrix *x_hat, struct Matrix Bd, double u, struct Matrix Ld, double y, struct Matrix C);

void Kalman_state_predict(struct Kalman_filter Kalman);

void Kalman_error_covariance_predict(struct Kalman_filter Kalman);

void Kalman_gain_computation(struct Kalman_filter Kalman);

void Kalman_state_estimation(struct Kalman_filter Kalman);

void Kalman_error_covariance_update(struct Kalman_filter Kalman);

void Kalman_filtering(struct Kalman_filter K);
