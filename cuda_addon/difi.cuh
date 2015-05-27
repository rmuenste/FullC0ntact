#include <stdio.h>

__global__ void my_kernel(){
  printf("Hello!\n");
}

extern "C" void my_cuda_func(){
  my_kernel <<<1, 1 >>>();
  cudaDeviceSynchronize();
}