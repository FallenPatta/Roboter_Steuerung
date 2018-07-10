#include <stdio.h>
#include <cuda.h>
#include <time.h>

#include <iostream>

#define HANDLE_ERROR( err ) ( HandleError( err, __FILE__, __LINE__ ) )

static void HandleError( cudaError_t err, const char *file, int line )
{
    if (err != cudaSuccess)
      {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
}

//#include <pcl/point_types.h>

typedef struct{
	float x,y,z,u;
}PointXYZ;

//~ struct Mat_Structure{
	//~ float _11_, _21_, _31_, _41_;
	//~ float _12_, _22_, _32_, _42_;
	//~ float _13_, _23_, _33_, _43_;
	//~ float _14_, _24_, _34_, _44_;
//~ };

__global__
void kernel(float *vec, float *mat, float *out, int problem_size){
    int tid = blockDim.x * blockIdx.x + threadIdx.x; //threadIdx.x;
	
	if(tid < problem_size)
	{
		#pragma unroll
		for(int i = 0; i<4; i++)
		{
			out[4*tid+i] = 0;
		}
		
		#pragma unroll
		for(int i=0; i<4; i++)
		{
			#pragma unroll
			for(int j=0; j<4; j++)
			{
				out[4*tid+i] += vec[4*tid+j] * mat[4*i+j];
			}
		}
	}
}


__global__
void transform_kernel(PointXYZ *vec, float *mat, int problem_size){
    int tid = blockDim.x * blockIdx.x + threadIdx.x; //threadIdx.x;
	
	if(tid < problem_size)
	{
		float x = 0;
		float y = 0;
		float z = 0;
		
		x = vec[tid].x * mat[0] + vec[tid].y * mat[1] + vec[tid].z * mat[2] + mat[3];
		y = vec[tid].x * mat[4] + vec[tid].y * mat[5] + vec[tid].z * mat[6] + mat[7];
		z = vec[tid].x * mat[8] + vec[tid].y * mat[9] + vec[tid].z * mat[10] + mat[11];
		
		vec[tid].x = x;
		vec[tid].y = y;
		vec[tid].z = z;
	}
}


int testmain(float* vector_array, float* result_array, const int array_size, float* mat_4x4) {
	float *dev_array, *dev_mat, *dev_result;

	cudaMalloc((void**)&dev_array, sizeof(float)*4*array_size);
	cudaMalloc((void**)&dev_mat, sizeof(float)*16);
	cudaMalloc((void**)&dev_result, sizeof(float)*4*array_size);

	cudaMemcpy(dev_array, vector_array, sizeof(float)*4*array_size, cudaMemcpyHostToDevice);
	cudaMemcpy(dev_mat, mat_4x4, sizeof(float)*16, cudaMemcpyHostToDevice);

    //~ printf("\n\nRunning Kernel...\n\n");
	//kernel<<<1, array_size>>>(dev_array, dev_mat, dev_result);
	// Invoke kernel
	int threadsPerBlock = 1024;
	int blocksPerGrid = (array_size + threadsPerBlock - 1) / threadsPerBlock;
	kernel<<<blocksPerGrid, threadsPerBlock>>>(dev_array, dev_mat, dev_result, array_size);

	cudaMemcpy(result_array, dev_result, sizeof(float)*4*array_size, cudaMemcpyDeviceToHost);

	cudaFree(dev_array);
	cudaFree(dev_mat);
	cudaFree(dev_result);

	return 0;
};


int point_transform(PointXYZ * points, const int array_size, float* mat_4x4) {
	
	//~ printf("\n\nRunning Kernel...\n\n");
	
	PointXYZ *dev_array;
	float *dev_mat;

	HANDLE_ERROR(cudaMalloc(&dev_array, sizeof(PointXYZ)*array_size));
	HANDLE_ERROR(cudaMalloc(&dev_mat, sizeof(float)*16));
	
	//~ cudaMalloc((void**)&dev_array, sizeof(PointXYZ)*array_size);
	//~ cudaMalloc((void**)&dev_mat, sizeof(float)*16);

	//~ std::cout << "cudalib: (" << points[0].x << "," << points[0].y << "," << points[0].z << ")" << std::endl;

	HANDLE_ERROR(cudaMemcpy(dev_array, points, sizeof(PointXYZ)*array_size, cudaMemcpyHostToDevice));
	HANDLE_ERROR(cudaMemcpy(dev_mat, mat_4x4, sizeof(float)*16, cudaMemcpyHostToDevice));

	// Invoke kernel
	//~ int threadsPerBlock = 128;
	//~ int blocksPerGrid = (array_size + threadsPerBlock - 1) / threadsPerBlock;
	//~ transform_kernel<<<blocksPerGrid, threadsPerBlock>>>(dev_array, dev_mat, array_size);
	transform_kernel<<<16, 128>>>(dev_array, dev_mat, array_size);

	HANDLE_ERROR(cudaMemcpy(points, dev_array, sizeof(PointXYZ)*array_size, cudaMemcpyDeviceToHost));

	//~ std::cout << "cudalib after: (" << points[0].x << "," << points[0].y << "," << points[0].z << ")" << std::endl;

	cudaFree(dev_array);
	cudaFree(dev_mat);

	return 0;
};

int point_transform3(PointXYZ * points, const int array_size, PointXYZ * result_points, float* mat_4x4) {
	PointXYZ *dev_array;
	float *dev_mat;

	cudaMalloc((void**)&dev_array, sizeof(PointXYZ)*array_size);
	cudaMalloc((void**)&dev_mat, sizeof(float)*16);

	cudaMemcpy(dev_array, points, sizeof(PointXYZ)*array_size, cudaMemcpyHostToDevice);
	cudaMemcpy(dev_mat, mat_4x4, sizeof(float)*16, cudaMemcpyHostToDevice);

	// Invoke kernel
	int threadsPerBlock = 1024;
	int blocksPerGrid = (array_size + threadsPerBlock - 1) / threadsPerBlock;
	transform_kernel<<<blocksPerGrid, threadsPerBlock>>>(dev_array, dev_mat, array_size);

	cudaMemcpy(result_points, dev_array, sizeof(PointXYZ)*array_size, cudaMemcpyDeviceToHost);

	cudaFree(dev_array);
	cudaFree(dev_mat);

	return 0;
};

