#include <omp.h>//调用Openmp库函数
#include <iostream>

#define N 6

int main(int argc, char *argv[])
{
  int i;
  printf ("*Hello World! Thread: %d\n",
         omp_get_thread_num());
  std::cout <<"核"<<omp_get_max_threads()<< '\n';
  #pragma omp parallel for
    for (i = 0; i < N; ++i)
      printf ("Hello World!  Thread: %d, i: %d\n",
              omp_get_thread_num(), i);
   return 0;
}
