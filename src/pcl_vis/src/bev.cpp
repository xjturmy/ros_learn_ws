#include "preprocess.h"
#include <assert.h>
#include <iostream>
#include <math.h>

PreProcessCuda::PreProcessCuda()
{

    return;
}

PreProcessCuda::~PreProcessCuda()
{
    // checkCudaErrors(cudaFree(mask_));

    return;
}

void PreProcessCuda::generateBevProjection(float *points, size_t points_size, float *bevImage)
{
    // float min_x_range = params_.min_x_range;
    // float max_x_range = params_.max_x_range;
    // float min_y_range = params_.min_y_range;
    // float max_y_range = params_.max_y_range;
    // float min_z_range = params_.min_z_range;
    // float max_z_range = params_.max_z_range;

    float min_x_range = -5;
    float max_x_range = 5;
    float min_y_range = -5;
    float max_y_range = 5;
    float min_z_range = -5;
    float max_z_range = 5;

    float voxel_x_size = 0.1;
    float voxel_y_size = 0.1;

    int proj_x_size = 0.5;
    int proj_y_size = 0.5;

    float z_value_min = -0.5;
    float z_value_max = 2;
    int i_value_min = -2;
    int i_value_max = 2;
    int r_value_min = -2;
    int r_value_max = 2;

    int nnn = 0;

    for (size_t i = 0; i < points_size; ++i)
    {
        float4 point = ((float4 *)points)[i];
        // float point_r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z) * point.w;
        // std::cerr << i << ": "<< point.x << " " << point.y << " " << point.z << " " << point.w << " "<< point_r << std::endl;
        if (point.x > min_x_range && point.x < max_x_range &&
            point.y > min_y_range && point.y < max_y_range &&
            point.z > min_z_range && point.z < max_z_range)
        {
            float point_r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z) * point.w;

            int voxel_idx = floorf((point.x - min_x_range) / voxel_x_size);
            int voxel_idy = floorf((point.y - min_y_range) / voxel_y_size);
            nnn += 1;
            // std::cerr << nnn << ": "<< point.z << " "<< point.w << " " << point_r << " " << voxel_idx << " " << voxel_idy << std::endl;

            // unsigned int voxel_index = voxel_idy * proj_x_size + voxel_idx;
            unsigned int voxel_index = voxel_idx * proj_x_size + voxel_idy;

            if (point.z < z_value_min)
                point.z = z_value_min;
            if (point.z > z_value_max)
                point.z = z_value_max;
            point.z = (point.z - z_value_min) / (z_value_max - z_value_min);

            if (point.w < i_value_min)
                point.w = i_value_min;
            if (point.w > i_value_max)
                point.w = i_value_max;
            point.w = (point.w - i_value_min) / (i_value_max - i_value_min);

            if (point_r < r_value_min)
                point_r = r_value_min;
            if (point_r > r_value_max)
                point_r = r_value_max;
            point_r = (point_r - r_value_min) / (r_value_max - r_value_min);
            // std::cerr << nnn << ": "<< point.z << " "<< point.w << " " << point_r << " " << voxel_idx << " " << voxel_idy << " " << voxel_index << std::endl;

            bevImage[proj_x_size * proj_x_size * 0 + voxel_index] = point.z;
            bevImage[proj_x_size * proj_x_size * 1 + voxel_index] = point.w;
            bevImage[proj_x_size * proj_x_size * 2 + voxel_index] = point_r;
        }
    }
}
