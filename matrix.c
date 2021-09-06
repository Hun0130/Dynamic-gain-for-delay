#include "matrix.h"
//Sampling time [s]

// get ||n||
double abs_double(double n){
	if (n>0)
		return n;
	else
		return -n;	
}

// Get n!
int factorial(int n)
{
    if (n == 1)     
        return 1;    

    return n * factorial(n - 1);   
}

// Make a matrix
struct Matrix Init_Mat(int num_row, int num_col, double A[][num_col]){
	struct Matrix result;
	int i, j;

	result.mat = malloc(sizeof(double *) * num_row);

	for(i = 0; i < num_row; i++){
        *(result.mat+i) = malloc(sizeof(double) * num_col);
	}

	for(i = 0; i < num_row; i++){
		for(j = 0; j < num_col; j++)
			result.mat[i][j] = *(*(A+i)+j);
	}

	result.num_row = num_row;
	result.num_col = num_col;

	return result;
}

// Make a matrix (2)
struct Matrix Init_Mat_dptr(int num_row, int num_col, double** A){
	struct Matrix result;
	int i, j;

	result.mat=malloc(sizeof(double *) * num_row);

	for(i = 0; i < num_row; i++){
        *(result.mat+i) = malloc(sizeof(double) * num_col);
	}

	for(i = 0; i < num_row; i++){
		for(j = 0; j < num_col; j++)
			result.mat[i][j] = *(*(A + i) + j);
	}

	result.num_row = num_row;
	result.num_col = num_col;

	return result;
}

// Multiply the matrix A and B
struct Matrix mat_mul(struct Matrix A, struct Matrix B){
	struct Matrix result;
	int i, j, k;
	int temp_num_row = A.num_row;
	int temp_num_col = B.num_col;

  // A's column number == B's row number
	if(A.num_col != B.num_row){
		perror("row # of A and col # of B are not equal");
		return result;
	}

	else{
		result.mat=malloc(sizeof(double *) * A.num_row);

  // Repeat by row size
		for(i = 0; i < A.num_row; i++){            
      // Dynamic memory allocation by (int size * column size).
      *(result.mat+i) = malloc(sizeof(double) * B.num_col);    
    }

    // all matrix values to zero
    for(i = 0; i < temp_num_row; i++){
      for(j = 0; j < temp_num_col; j++){
        result.mat[i][j]=0;
      }
    }

    // multipy A and B
    for(i = 0; i < temp_num_row; i++){
      for(j = 0; j < temp_num_col; j++){
        for(k = 0; k < A.num_col; k++){
          result.mat[i][j]+= A.mat[i][k] * B.mat[k][j];
        }
      }
    }
    result.num_row = temp_num_row;
    result.num_col = temp_num_col;
    return result;
	}
}

// Multyply scalar value to the matrix A
struct Matrix mat_mul_scalar(struct Matrix A, double c){
  struct Matrix result;
  int i, j;
  int temp_num_row = A.num_row;
  int temp_num_col = A.num_col;
  result.mat = malloc(sizeof(double *) * A.num_row);

  for(i = 0; i < A.num_row; i++){            
      *(result.mat+i) = malloc(sizeof(double) * A.num_col);
  }

  for(i = 0; i < temp_num_row; i++){
    for(j = 0; j < temp_num_col; j++){
      result.mat[i][j] = A.mat[i][j] * c;
    }
  }
  result.num_row = temp_num_row;
  result.num_col = temp_num_col;
  return result;
}

// Transpose the matrix A to A^T
struct Matrix mat_transpose(struct Matrix A){
  int num_row = A.num_row;
  int num_col = A.num_col;
  int i, j;
  struct Matrix AT;

  AT.num_row = num_col;
  AT.num_col = num_row;

  AT.mat=malloc(sizeof(double *) * AT.num_row);

  for(i = 0; i < AT.num_row; i++){
    *(AT.mat + i) = malloc(sizeof(double) * AT.num_col);
  }

  for(i = 0; i < num_col; i++){
    for(j = 0; j < num_row; j++){
      AT.mat[i][j]=A.mat[j][i];
    }
  }
  return AT;
}

// Copy the matrix A to B,  A: original matrix, B: empty matrix
void mat_cpy(struct Matrix *A, struct Matrix *B){ 
	int i, j;

	if(A->num_row!=B->num_row || A->num_col!=B->num_col)
		perror("Sizes of matricies are not equal");
	else{
		for(i=0; i<B->num_row; i++){
			for(j=0; j<B->num_col; j++)
				B->mat[i][j]=A->mat[i][j];
		}
	}
}

// Print the matrix A
void mat_print(struct Matrix A){
  int i, j;
  for(i = 0; i < A.num_row; i++){
    for(j = 0; j < A.num_col; j++){
      printf("%lf\t", A.mat[i][j]);
    }
  }
}

// unallocate memory of the matrix A
void Matrix_free(struct Matrix A){
	int row=A.num_row;
	int i;
	for(i=0; i<row; i++){
		free(A.mat[i]);
	}
	free(A.mat);
}

//Get determinent of matrix A
double mat_det(struct Matrix A){ 
  if (A.num_col != A.num_row){
    printf("Dimension of row(A) is %d and col(A) is %d.\nDimension of row and column are not equal. Not square matrix!!!\n", A.num_row, A.num_col);
    exit(0);
  }
  int dim = A.num_col;
  int i, j, k;
  double result = 1;
  double** dptr_B;
  double** dptr_I;
  struct Matrix B,I;

  if (dim == 1)
    result= A.mat[0][0];

  else if (dim == 2)
    result = (A.mat[0][0] * A.mat[1][1] - A.mat[1][0] * A.mat[0][1]);
  
  //Faddeev-Leverrier Algorithm
  else{ 
    //Matrix initializatino
    dptr_B = malloc(sizeof(double *)* dim);
    dptr_I = malloc(sizeof(double *)* dim);

    for(i = 0; i < dim; i++){
      *(dptr_B + i) = malloc(sizeof(double)*dim);
      *(dptr_I + i) = malloc(sizeof(double)*dim);
    }

    B = Init_Mat_dptr(dim,dim,dptr_B);
    I = Init_Mat_dptr(dim,dim,dptr_I);

    for (i = 0; i < dim; i++){
      for(j = 0; j < dim; j++){
        B.mat[i][j] = 0;
        if(i == j)
          I.mat[i][j] = 1;
        else
          I.mat[i][j] = 0;
      }
    }

    //Faddeev-Leverrrier Algorithm excution
    for(i = 0; i < dim; i++){
      for(j = 0; j < dim; j++){
        for(k = 0; k < dim; k++){
          B.mat[j][k] += result * I.mat[j][k];
        }
      }
      B = mat_mul(A,B);
      result =- mat_trace(B)/(i+1);
    }
    if(dim % 2 == 1)
      result =- result;
    //Free the memory
    for(i = 0; i < dim; i++){
      free(*(dptr_B + i));
      free(*(dptr_I + i));
    }
    free(dptr_B);
    free(dptr_I);
    Matrix_free(B); Matrix_free(I);
  }
  return result;
}

struct Matrix mat_inv(struct Matrix A){
  int dim= A.num_col;
  int i, j, k;
  double** ptr;
  double** ptr_result;
  struct Matrix A_input, temp, result;
  A_input=A;
  double det;
  double coefficient;

  mat_print(A_input);

  /*det=mat_det(A_input);
  printf("determinent in A_input: %lf \n",det);
*/

  if (A.num_col != A.num_row){
    printf("Dimension of row(A) is %d and col(A) is %d.\nDimension of row and column are not equal. Not square matrix!!!\n", A.num_row, A.num_col);
    exit(0);
  }

  if(det==0){
    printf("This matrix is singular. It's impossible to get inverse matrix\n");
    exit(0);
  }

  ptr = malloc(sizeof(double*) * (dim));
  for(i = 0; i < dim; i++){
        ptr[i] = malloc(sizeof(double) * dim*2);
        for(j=0; j<2*dim; j++){
          ptr[i][j]=0;
        }
  }
  /*
  for (i=0; i< dim; i++){
    for(j=0; j<dim*2; j++){
      *(*(ptr+i)+j) = 0;
    }
  }
*/
  ptr_result= malloc(sizeof(double*)*dim);
  for(i=0; i<dim; i++){
    ptr_result[i]=malloc(sizeof(double)*dim);
    for(j=0; j<dim; j++){
      ptr_result[i][j]=0;
    }
  }

  result=Init_Mat_dptr(dim, dim,ptr_result);

  temp=Init_Mat_dptr(dim,2*dim,ptr);

  for (i=0; i< dim; i++){
    for(j=0; j<dim; j++){
      temp.mat[i][j] = A_input.mat[i][j];
    }
  }

  for(i=0; i<dim; i++){
      temp.mat[i][i+dim] = 1.0;
  }

  
  for(i=0; i<dim; i++){
    for(j=i+1; j<dim; j++){ //#of row
      if(temp.mat[i][i] != 0){
        coefficient=temp.mat[j][i]/temp.mat[i][i];
        for(k=0; k<2*dim; k++){
          temp.mat[j][k]=temp.mat[j][k]-coefficient*temp.mat[i][k];
        }
      }
    }
  }

  for(i=0; i<dim; i++){
    for(j=i+1; j<dim; j++){ //#of row
      if(temp.mat[dim-1-i][dim-1-i] != 0){
        coefficient=temp.mat[dim-1-j][dim-1-i]/temp.mat[dim-1-i][dim-1-i];
        for(k=0; k<2*dim; k++){
          temp.mat[dim-1-j][k]=temp.mat[dim-1-j][k]-coefficient*temp.mat[dim-1-i][k];
        }
      }
    }
  }
  for (i=0; i<dim; i++){
    coefficient=temp.mat[i][i];
    if (coefficient == 0){
      printf("Error: This matrix is singular.\n");
      exit(0);
    }
    else{
      for (j=0; j<2*dim; j++){
        temp.mat[i][j]=temp.mat[i][j]/coefficient;
      }
    }
  }

  mat_cpy(&A_input,&result);

  for(i=0; i<dim; i++){
    for(j=0; j<dim; j++){
      result.mat[i][j]=temp.mat[i][dim+j];
    }
  }


  for(i = 0; i < dim; i++){
        free(*(ptr+i));
        free(*(ptr_result+i));
  }
  free(ptr);
  free(ptr_result);
  //Matrix_free(A_input);
  Matrix_free(temp);
  return result;
}

double mat_trace(struct Matrix A){
  int i, j, dim;
  double result=0;
  if(A.num_col!=A.num_row){
    printf("Error: Not square matrix!\n");
    exit(0);
  }
  dim= A.num_col;

  for(i=0; i<dim; i++){
    result+=A.mat[i][i];
  }
  return result;
}

struct Matrix get_Transition_Matrix(struct Matrix A, unsigned int iter, double dtime){
  double eye[DIMENSION][DIMENSION]={0,};
  double zero[DIMENSION][DIMENSION]={0,};
  double temp_dtime=1;
  struct Matrix result, temp, temp2;
  int i, j, k;

  for(i=0; i<DIMENSION; i++){
    for(j=0; j<DIMENSION; j++){
      if(i==j)
        eye[i][j]=1;
    }
  }
  result=Init_Mat(DIMENSION,DIMENSION,eye);
  temp=Init_Mat(DIMENSION,DIMENSION,eye);
  temp2=Init_Mat(DIMENSION,DIMENSION,eye);

if (dtime==0) {
  return result;
}

for(i=1; i<iter+1; i++){  //iteration count
  temp=mat_mul(temp2,A);
  temp_dtime=temp_dtime*dtime;
  mat_cpy(&temp,&temp2);

  for(j=0; j<DIMENSION; j++){
    for(k=0; k<DIMENSION; k++){
      temp2.mat[j][k]=temp.mat[j][k]*temp_dtime/factorial(i);
    }
  }

  for(j=0; j<DIMENSION; j++){ //Transition_matrix update
    for(k=0; k<DIMENSION; k++){
      result.mat[j][k]+=temp2.mat[j][k];
    }
  }
  //mat_print(result);
  mat_cpy(&temp, &temp2);
}
  Matrix_free(temp); Matrix_free(temp2);

  return result;
}

struct Matrix get_Bd(struct Matrix A, struct Matrix B, double dtime){
  int iter=10;
  struct Matrix result, temp, temp_B;
  double zero[DIMENSION][DIMENSION]={0,};
  int i, j, k;

  //result=Init_Mat(DIMENSION,DIMENSION,zero);
  temp=Init_Mat(DIMENSION,DIMENSION,zero);
  result=Init_Mat(DIMENSION,NUM_INPUT,zero);
  temp_B=Init_Mat(DIMENSION,NUM_INPUT,zero);

  for(i=0; i<dtime/EPSILON; i++){
    temp=get_Transition_Matrix(A,iter,i*EPSILON);
    for(j=0;j<DIMENSION;j++){
      for(k=0; k<NUM_INPUT; k++){
        temp_B.mat[j][k]=mat_mul(temp,B).mat[j][k]*EPSILON;
      }
    }

    for(j=0;j<DIMENSION;j++){
      for(k=0; k<NUM_INPUT; k++){
        result.mat[j][k]+=temp_B.mat[j][k];
      }
    }

  }
  return result;
}

// update state : x' = Ax + Bu
void update_state(struct Matrix A, struct Matrix *x, struct Matrix B, double u){
  double zero[DIMENSION][1] = {0,};
  struct Matrix x_dot;
  int i, j;

  x_dot = Init_Mat(DIMENSION, 1, zero);
  // x' = Ax
  x_dot = mat_mul(A, *x);
  // x' = Ax + Bu
  for(i = 0; i < DIMENSION; i++){
    for(j = 0; j < 1; j++){
      x_dot.mat[i][j] += mat_mul_scalar(B,u).mat[i][j];
    }
  }
  // multiply delta 
  for(i = 0; i < DIMENSION; i++){
    for(j = 0; j < 1; j++){
      x -> mat[i][j] += x_dot.mat[i][j] * DELTA;
    }
  }

}

// update y = Cx + D
double update_y(struct Matrix C, struct Matrix *x){
  double y;
  struct Matrix y_mat;
  y_mat = mat_mul(C,*x);
  y = y_mat.mat[0][0];
  return y;
}

double update_u(struct Matrix K, struct Matrix *x_hat){
  double u;
  struct Matrix u_mat;
  u_mat = mat_mul(K, *x_hat);
  u = u_mat.mat[0][0];

  return -u;
}

void update_x_hat(struct Matrix Ad, struct Matrix *x_hat, struct Matrix Bd, double u, struct Matrix Ld, double y, struct Matrix C){
  //double zero[DIMENSION][1]={0,};
  double e;
  struct Matrix x_next;
  int i, j;

  x_next=mat_mul(Ad,*x_hat);

  for(i=0; i< DIMENSION; i++){
    for(j=0; j<1; j++){
      x_next.mat[i][j]+=mat_mul_scalar(Bd,u).mat[i][j];
    }
  }

  e=y-update_y(C,x_hat);
  for(i=0; i< DIMENSION; i++){
    for(j=0; j<1; j++){
      x_next.mat[i][j]+=mat_mul_scalar(Ld,e).mat[i][j];
    }
  }

  for(i=0; i< DIMENSION; i++){
    for(j=0; j<1; j++){
      x_hat->mat[i][j]=x_next.mat[i][j];
    }
  }
}

void Kalman_state_predict(struct Kalman_filter Kalman){
	*(Kalman.x_kalman_temp)=mat_mul(*(Kalman.A),*(Kalman.x_kalman));
}

void Kalman_error_covariance_predict(struct Kalman_filter Kalman){
	int i,j;
	*(Kalman.P_temp)=mat_mul(mat_mul(*(Kalman.A),*(Kalman.P)),mat_transpose(*(Kalman.A)));
	for(i=0; i<DIMENSION; i++){
		(Kalman.P_temp)->mat[i][i]+=*(Kalman.cov_v);
	}
}

void Kalman_gain_computation(struct Kalman_filter Kalman){
	int i;
	double denominator=0;
	struct Matrix den_temp, den_temp2;
	
	den_temp=mat_mul(*(Kalman.C),*(Kalman.P_temp));
	den_temp2=mat_mul(den_temp, mat_transpose(*(Kalman.C)));
	denominator=den_temp2.mat[0][0];
	denominator+=(*(Kalman.cov_w))*(*(Kalman.cov_w));

	if(denominator==0){
		printf("Error:denominator is zero in function of Kalman gain computation");
		exit(0);
	}

	*(Kalman.Kalman_gain)=mat_mul(*(Kalman.P_temp),mat_transpose(*(Kalman.C)));
	*(Kalman.Kalman_gain)=mat_mul_scalar(*(Kalman.Kalman_gain),1/denominator);
	Matrix_free(den_temp);Matrix_free(den_temp2);
}

void Kalman_state_estimation(struct Kalman_filter Kalman){
	int i;
	double estimation_error;
	estimation_error=*(Kalman.y)-mat_mul(*(Kalman.C),*(Kalman.x_kalman_temp)).mat[0][0];
	for(i=0; i<DIMENSION; i++)
	(Kalman.x_kalman)->mat[i][0]=(Kalman.x_kalman_temp)->mat[i][0]+mat_mul_scalar(*(Kalman.Kalman_gain),estimation_error).mat[i][0];
}

void Kalman_error_covariance_update(struct Kalman_filter Kalman){
	int i, j;
	struct Matrix temp;
	temp=mat_mul(*(Kalman.Kalman_gain),*(Kalman.C));
	for(i=0; i<DIMENSION; i++){
		for(j=0; j<DIMENSION; j++){
			if(i==j)
				temp.mat[i][j]=1-temp.mat[i][j];
			else
				temp.mat[i][j]=-temp.mat[i][j];
		}
	}

	*(Kalman.P)=mat_mul(temp,*(Kalman.P_temp));
}

void Kalman_filtering(struct Kalman_filter K){
	struct Matrix tempk;
	tempk=mat_transpose(*(K.C));
	Kalman_state_predict(K);
	Kalman_error_covariance_predict(K);
	Kalman_gain_computation(K);
	Kalman_state_estimation(K);
	Kalman_error_covariance_update(K);
}

