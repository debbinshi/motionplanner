#pragma once

#include <math.h>
#include <vector> 
#include "geometry.h"
#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"
#include <vector>
#include <ceres/ceres.h>
#include "U_costfunction.h"
#include "posi_costfunction.h"
#include "obst_costfunction.h"
#define pn_ 10
#define cn_ 2
#define pn_3 30
#define cn_2 4
#define pn_obstn 20

using namespace Eigen;

class mpc_planner {
public:
#define WHEEL_BASE 2.8498f

    mpc_planner();
    ~mpc_planner();

    bool init(double dt);

    bool plan(pos3d_t ego_pos, std::vector<pos3d_t> &ref_traj, std::vector<pos3d_t> &todo_traj, std::vector<pos3d_t> &obst_pos);

private:
    double dt_;
    int ref_cnt_;
    int ref_idx_;

    MatrixXd Q_cell_;
    MatrixXd R_cell_;

    double acc_max_;
    double acc_min_;
    double delta_max_;
    double delta_min_;

    double Sobs_;
    double kesi_;    
    
    double* A_delta_;
    std::vector<double> time;
};
