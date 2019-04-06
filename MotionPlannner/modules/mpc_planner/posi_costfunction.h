#pragma once

#include "Eigen/Dense"
#include <ceres/ceres.h>
#include <vector>
using namespace std;

using namespace Eigen;
class posi_costfunction{
public:    
    posi_costfunction(vector<vector<double>> &B_cell_tmp, vector<vector<double>> &Q_cell_tmp, vector<double> &y_tmp) :
    B_cell_tmp_(B_cell_tmp), Q_cell_tmp_(Q_cell_tmp), y_tmp_(y_tmp){}

    template <typename T>
    bool operator()(const T* const U, T* residual)const{

//      MatrixXd residual_mat = Q_cell_tmp_ * (A_cell_tmp_ * data_ + B_cell_tmp_ * U_matrix- Yref); 3 * pn_
//        MatrixXd y_tmp = A_cell_tmp_ * data_ - Yref;
        for(unsigned int i = 0; i < B_cell_tmp_.size(); i++){
            residual[i] = T(0.0);
            for(unsigned int j = 0; j < B_cell_tmp_[0].size(); j++){
                residual[i] += T(B_cell_tmp_[i][j]) * U[j];
            }
            residual[i] = ceres::sqrt(Q_cell_tmp_[i][i]) * (T(y_tmp_[i]) + residual[i]);
            cout << residual[i] << endl;
        }
        for(unsigned int j = 0; j < B_cell_tmp_[0].size(); j++)
            cout << U[j] << endl;

        return true;
    }

private:
    vector<vector<double>> B_cell_tmp_;
    vector<vector<double>> Q_cell_tmp_;
    vector<double> y_tmp_;
};
