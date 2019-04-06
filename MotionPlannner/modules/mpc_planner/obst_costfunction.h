#pragma once

#include "Eigen/Dense"
#include <ceres/ceres.h>
#include "math.h"

using namespace Eigen;
class obst_costfunction{
public:    
    obst_costfunction(vector<vector<double>> &B_cell_tmp, vector<double> &x_pre0, vector<double> &y_pre0, vector<double> &vel_pre0, vector<double> &x_obst, vector<double> &y_obst, double kesi, double S_obst) :
    B_cell_tmp_(B_cell_tmp), x_pre0_(x_pre0), y_pre0_(y_pre0), vel_pre0_(vel_pre0), x_obst_(x_obst), y_obst_(y_obst), kesi_(kesi), S_obst_(S_obst){}

    template <typename T>
    bool operator()(const T* const U, T* residual)const{

//      residual = sqrt(Sobs) * sum(vk / ((x_pre_(k) - x_obst_(k, j)) ^ 2 + (y_pre_(k) - y_obst_(k, j)) ^ 2));  Np * Nobst

//      MatrixXd state_pre = A_state_tmp_ * cur_err + B_cell_tmp_ * U + refs;
        for(unsigned int i = 0; i < x_pre0_.size(); i++){

            T x_pre = T(x_pre0_[i]);
            T y_pre = T(y_pre0_[i]);
            T vel_pre = T(vel_pre0_[i]);
            for(unsigned int k = 0; k < B_cell_tmp_[0].size(); k++){
               x_pre += T(B_cell_tmp_[3 * i][k]) * U[k];                
               y_pre += T(B_cell_tmp_[3 * i + 1][k]) * U[k];                
               vel_pre += T(B_cell_tmp_[3 * i + 2][k]) * U[k];                
            }

            for(unsigned int j = 0; j < x_obst_.size(); j++){
                residual[i * x_obst_.size() + j] = T(ceres::sqrt(S_obst_)) * vel_pre / T(ceres::sqrt(ceres::pow((x_pre - x_obst_[j]), 2) + ceres::pow((y_pre - y_obst_[j]), 2) + kesi_));
                cout << residual[i * x_obst_.size() + j] << endl;
            }

            for(unsigned int k = 0; k < B_cell_tmp_[0].size(); k++){
                cout << U[k] << endl;
            }
        }
        return true;
    }

private:
    vector<vector<double>> B_cell_tmp_;
    vector<double> x_pre0_;
    vector<double> y_pre0_;
    vector<double> vel_pre0_;

    vector<double> x_obst_;
    vector<double> y_obst_;
    double kesi_;
    double S_obst_;
};
