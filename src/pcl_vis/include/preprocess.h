#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "kernel.h"

class PreProcessCuda
{
private:
    Params params_;
    cudaStream_t stream_ = 0;
    unsigned int *mask_;

public:
    PreProcessCuda();
    ~PreProcessCuda();

    void generateBevProjection(float *points, size_t points_size, float *bevimage);
};

#endif
