#include <stdio.h>
#include <cuda.h>
#include <time.h>

//#include <pcl/point_types.h>

struct PointXYZ{
	float x,y,z,u;
};

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

//~ __global__
//~ void transform_kernel2(PointXYZ *vec, Mat_Structure mat, int problem_size){
    //~ int tid = blockDim.x * blockIdx.x + threadIdx.x; //threadIdx.x;
	
	//~ if(tid < problem_size)
	//~ {
		//~ float x = 0;
		//~ float y = 0;
		//~ float z = 0;
		
		//~ x = vec[tid].x * mat._11_ + vec[tid].y * mat._12_ + vec[tid].z * mat._13_ + mat._14_;
		//~ y = vec[tid].x * mat._21_ + vec[tid].y * mat._22_ + vec[tid].z * mat._23_ + mat._24_;
		//~ z = vec[tid].x * mat._31_ + vec[tid].y * mat._32_ + vec[tid].z * mat._33_ + mat._34_;
		
		//~ vec[tid].x = x;
		//~ vec[tid].y = y;
		//~ vec[tid].z = z;
	//~ }
//~ }


int testmain(float* vector_array, float* result_array, const int array_size, float* mat_4x4) {
	float *dev_array, *dev_mat, *dev_result;

	cudaMalloc((void**)&dev_array, sizeof(float)*4*array_size);
	cudaMalloc((void**)&dev_mat, sizeof(float)*16);
	cudaMalloc((void**)&dev_result, sizeof(float)*4*array_size);

	cudaMemcpy(dev_array, vector_array, sizeof(float)*4*array_size, cudaMemcpyHostToDevice);
	cudaMemcpy(dev_mat, mat_4x4, sizeof(float)*16, cudaMemcpyHostToDevice);

    printf("\n\nRunning Kernel...\n\n");
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

	cudaMemcpy(points, dev_array, sizeof(PointXYZ)*array_size, cudaMemcpyDeviceToHost);

	cudaFree(dev_array);
	cudaFree(dev_mat);

	return 0;
};

