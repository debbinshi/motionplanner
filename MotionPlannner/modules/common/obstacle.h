#pragma once

#include "geometry.h"

typedef struct {
    pos3d_t pos;
    std::vector<pos3d_t> old_traj;
    std::vector<pos3d_t> new_traj;
    double timestamp_ms;
}car_t;

typedef struct {
    pos3d_t pos;
    std::vector<pos3d_t> old_traj;
    std::vector<pos3d_t> new_traj;
    double timestamp_ms;
}ped_t;

typedef struct {
    std::vector<pos3d_t> fs_pts;
    double timestamp_ms;
}fs_t;
