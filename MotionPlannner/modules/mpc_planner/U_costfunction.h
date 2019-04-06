#pragma once

#include "Eigen/Dense"
#include <ceres/ceres.h>

using namespace Eigen;
using namespace std;

class U_costfunction{
public:    
    U_costfunction(vector<vector<double>> &R_cell_tmp) : R_cell_tmp_(R_cell_tmp){}

    template <typename T>
    bool operator()(const T* const U, T* residual)const{

//        MatrixXd residual_mat = sqrt(R_cell_tmp) * (U); (2 * cn_, 1)
        for(unsigned int i = 0; i < R_cell_tmp_.size(); i++){
            residual[i] = T(ceres::sqrt(R_cell_tmp_[i][i])) * U[i];
            cout << residual[i] << endl;
            cout << U[i] << endl;
        }
        return true;
    }

private:
    vector<vector<double>> R_cell_tmp_;
};
