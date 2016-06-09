#ifndef MANAGED_HPP_MFELPDOV
#define MANAGED_HPP_MFELPDOV

#include <cuda_runtime_api.h>
#include <cuda.h>
#include <string>
#include <stdio.h>

class Managed
{
public:

  // handles the allocation and creation of an object:
  // something like:
  // X *x = new X() ~ void *temp = operator new(sizeof(X)); X *x = (temp)X;
  void *operator new(size_t len)
  {
    void *ptr;
    cudaMallocManaged(&ptr, len);
    cudaDeviceSynchronize();
    return ptr;
  }

  // handles the deallocation of the object:
  // something like:
  // delete x ~ x->~X(); operator delete(static_cast<void*>(x));
  void operator delete(void *ptr)
  {
    cudaDeviceSynchronize();
    cudaFree(ptr);
  }

private:
  /* data */
};


#endif /* end of include guard: MANAGED_HPP_MFELPDOV */
