#include "mpc_planner.h"

mpc_planner::mpc_planner(){}

mpc_planner::~mpc_planner(){}

bool mpc_planner::init(double dt){

    dt_ = dt;
    ref_cnt_ = 0;
    ref_idx_ = 0;

    Q_cell_ = MatrixXd::Identity(3 * pn_, 3 * pn_);
    for(int i = 0; i < 3 * pn_; i++) {
        if(i % 3 == 2)
            Q_cell_(i, i) = 0.1;
    }
    R_cell_ = MatrixXd::Identity(2 * cn_, 2 * cn_);

    acc_max_ = 5.0;
    acc_min_ = -5.0;
    delta_max_ = 1.0;
    delta_min_ = -1.0;

    Sobs_ = 10.0;
    kesi_ = 1.0;
 
    A_delta_ = new double[cn_ * 2];

    for(unsigned int i = 0; i < cn_ * 2; i++){
        A_delta_[i] = 0.1;
    }
    time.resize(pn_);
    
    for(unsigned int i = 0; i < time.size(); i++){
        time[i] = i * dt_;
    } 
    return true;
}

bool mpc_planner::plan(pos3d_t ego_pos, std::vector<pos3d_t> &ref_traj, std::vector<pos3d_t> &todo_traj, std::vector<pos3d_t> &obst_pos) {
    
    ref_cnt_ = ref_traj.size();
    std::cout << "entering" << endl;
    ref_idx_ = get_nearest_index(ego_pos, ref_traj, 0, ref_cnt_ - 1);

    pos3d_t ref_pt;
    ref_pt.x = ref_traj[ref_idx_].x;
    ref_pt.y = ref_traj[ref_idx_].y;
    ref_pt.theta = ref_traj[ref_idx_].theta;
    ref_pt.v = ref_traj[ref_idx_].v;

    double ref_delta = 0.0;
    double ref_acc = 0.0;

    MatrixXd cur_err(4, 1);
    cur_err(0) = ego_pos.x - ref_pt.x;
    cur_err(1) = ego_pos.y - ref_pt.y;
    cur_err(2) = ego_pos.theta - ref_pt.theta;
    cur_err(3) = ego_pos.v - ref_pt.v;
   
    while(cur_err(2) >= M_PI)
        cur_err(2) -= 2 * M_PI;
    
    while(cur_err(2) <= -M_PI)
        cur_err(2) += 2 * M_PI;

    vector<double> x_obst;
    vector<double> y_obst;
    for(unsigned int i = 0; i < obst_pos.size(); i++){
        x_obst.push_back(obst_pos[i].x);
        y_obst.push_back(obst_pos[i].y);
    }

    MatrixXd A = MatrixXd::Zero(4, 4);
    A(0, 0) = 1;
    A(0, 2) = -ref_pt.v * sin(ref_pt.theta) * dt_;
    A(0, 3) = cos(ref_pt.theta) * dt_;

    A(1, 1) = 1;
    A(1, 2) = ref_pt.v * cos(ref_pt.theta) * dt_;
    A(1, 3) = sin(ref_pt.theta) * dt_;

    A(2, 2) = 1; 
    A(2, 3) = tan(ref_delta) / WHEEL_BASE;

    A(3, 3) = 1;

    MatrixPower<MatrixXd> Apow(A);

    MatrixXd B = MatrixXd::Zero(4, 2);
    B(0, 0) = cos(ref_pt.theta) * dt_;
    B(1, 0) = sin(ref_pt.theta) * dt_;
    B(2, 1) = ref_pt.v * dt_ / pow(cos(ref_delta), 2) / WHEEL_BASE;
    B(3, 0) = dt_;
    cout << "B is" << endl;
    cout << B << endl; 
    MatrixXd C = MatrixXd::Zero(3, 4);
    C(0, 0) = 1.0;
    C(1, 1) = 1.0;
    C(2, 2) = 1.0;

    MatrixXd A_state_cell = MatrixXd::Zero(4 * pn_, 4);
    for(int i = 0; i < pn_; i++){
        A_state_cell.block<4, 4>(i * 4, 0) = Apow(i + 1);
    }

    MatrixXd A_cell = MatrixXd::Zero(3 * pn_, 4);
    for(int i = 0; i < pn_; i++){
        A_cell.block<3, 4>(i * 3, 0) = C * Apow(i + 1);
    }

    MatrixXd B_cell = MatrixXd::Zero(3 * pn_, 2 * cn_);
    for(int i = 0; i < pn_; i++) {
        for(int j = 0; j < cn_; j++) {
            if(i < cn_)
            {
                if(j < i)
                    B_cell.block<3, 2>(i * 3, j * 2) = A_cell.block<3, 4>((i - j - 1) * 3, 0) * B;
                else if(j == i)
                    B_cell.block<3, 2>(i * 3, j * 2) = C * B;
            }
            else
            {
                    if(j == cn_ - 1)
                    {
                        for(int k = 0; k < i - j; k++)
                        {
                            B_cell.block<3, 2>(i * 3, j * 2) += A_cell.block<3, 4>(k * 3, 0) * B;
                        }
                            B_cell.block<3, 2>(i * 3, j * 2) += C * B;
                    }
                    else
                    {
                        B_cell.block<3, 2>(i * 3, j * 2) = A_cell.block<3, 4>((i - j - 1) * 3, 0) * B;
                    }
            }
        }
    }

    MatrixXd Yref = MatrixXd::Zero(3 * pn_, 1);
    for(int i = 0; i < pn_; i++){
        if(ref_idx_ + i < ref_cnt_){
            Yref(i * 3) = ref_traj[ref_idx_ + i].x - ref_pt.x;
            Yref(i * 3 + 1) = ref_traj[ref_idx_ + i].y - ref_pt.y;
            Yref(i * 3 + 2) = ref_traj[ref_idx_ + i].theta - ref_pt.theta;
        }
        else{
            Yref(i * 3) = ref_traj[ref_cnt_].x - ref_pt.x;
            Yref(i * 3 + 1) = ref_traj[ref_cnt_].y - ref_pt.y;
            Yref(i * 3 + 2) = ref_traj[ref_cnt_].theta - ref_pt.theta;
        }
    }

    MatrixXd E = A_cell * cur_err - Yref;
    for(int i = 0; i < pn_; i++){
        while(E(i * 3 + 2) >= M_PI){
            E(i * 3 + 2) -= 2 * M_PI;
        }

        while(E(i * 3 + 2) <= -M_PI){
            E(i * 3 + 2) += 2 * M_PI;
        }
    }
    
    ceres::Problem problem;
    std::cout << "constructing U_costfunction" << std::endl;
    std::vector<std::vector<double>> R_cell_tmp;
    R_cell_tmp.resize(R_cell_.rows());
    for(unsigned int i = 0; i < R_cell_.rows(); i++){
        for(unsigned int j = 0; j < R_cell_.cols(); j++){
            R_cell_tmp[i].push_back(R_cell_(i, j));
        }
    }
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<U_costfunction, cn_2, cn_2>(
            new U_costfunction (R_cell_tmp)
        ),
        nullptr,
        A_delta_
    );

    std::cout << "constructing posi_costfunction" << std::endl;
    std::vector<std::vector<double>> B_cell_tmp;
    B_cell_tmp.resize(B_cell.rows());
    for(unsigned int i = 0; i < B_cell.rows(); i++){
        for(unsigned int j = 0; j < B_cell.cols(); j++){
            B_cell_tmp[i].push_back(B_cell(i, j));
        }
    }

    std::vector<std::vector<double>> Q_cell_tmp;
    Q_cell_tmp.resize(Q_cell_.rows());
    for(unsigned int i = 0; i < Q_cell_.rows(); i++){
        for(unsigned int j = 0; j < Q_cell_.cols(); j++){
            Q_cell_tmp[i].push_back(Q_cell_(i, j));
        }
    }
    
    std::vector<double> y_tmp;
    for(unsigned int i = 0; i < E.rows(); i++){
        y_tmp.push_back(E(i));
    }

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<posi_costfunction, pn_3, cn_2>(
            new posi_costfunction (B_cell_tmp, Q_cell_tmp, y_tmp)
        ),
        nullptr,
        A_delta_
    );
   

    std::cout << "constructing obst_costfunction" << std::endl;
    std::vector<double> x_pre0, y_pre0, vel_pre0;
    MatrixXd state_pre = A_state_cell * cur_err;
    for(int i = 0; i < state_pre.size(); i++){
        if(i % 4 == 0){
            x_pre0.push_back(state_pre(i) + ref_pt.x);
        }
        else if(i % 4 == 1){
            y_pre0.push_back(state_pre(i) + ref_pt.y);
        }
        else if(i % 4 == 3){
            vel_pre0.push_back(state_pre(i) + ref_pt.v);
        }
    }
    
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<obst_costfunction, pn_obstn, cn_2>(
            new obst_costfunction (B_cell_tmp, x_pre0, y_pre0, vel_pre0, x_obst, y_obst, kesi_, Sobs_)
        ),
        nullptr,
        A_delta_
    );

    std::cout << "constructing constraints" << std::endl;
    for(int i = 0; i < cn_; i++){
        problem.SetParameterLowerBound(A_delta_, 2 * i, acc_min_);
        problem.SetParameterUpperBound(A_delta_, 2 * i, acc_max_);
        problem.SetParameterLowerBound(A_delta_, 2 * i + 1, delta_min_);
        problem.SetParameterUpperBound(A_delta_, 2 * i + 1, delta_max_);
    }

    std::cout << "optimization begin!" << std::endl; 
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    
    ceres::Solve( options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
 
    A_delta_[0] += ref_acc;
    A_delta_[1] += ref_delta;
   
    std::cout << A_delta_[0] << " " << A_delta_[1] << std::endl; 
    MatrixXd U_matrix = Map<MatrixXd>(A_delta_, cn_2, 1);  
    MatrixXd Y_matrix = A_cell * cur_err + B_cell * U_matrix;
   
    pos3d_t pos_tmp; 
    for(unsigned int i = 0; i < pn_; i++){
        pos_tmp.x = Y_matrix(i * 3) + ref_pt.x;
        pos_tmp.y = Y_matrix(i * 3 + 1) + ref_pt.y;
        pos_tmp.theta = Y_matrix(i * 3 + 2) + ref_pt.theta;
        if(todo_traj.size() <= i)
            todo_traj.push_back(pos_tmp);
        else
            todo_traj[i] = pos_tmp;
    }
    
    return true;
}
