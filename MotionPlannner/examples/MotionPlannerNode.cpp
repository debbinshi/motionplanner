#include "mpc_planner.h"
#include <ros/ros.h>
#include "geometry.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "MotionPlanner1");

    ros::NodeHandle nh;

    std::cout << "initializing!" << std::endl;
    mpc_planner my_planner;
    double dt = 1.0;
//    double np_set = 10;
//    double nc_set = 2;
    bool succ_init = my_planner.init(dt);

    pos3d_t my_pos;
    my_pos.x = 0.0;
    my_pos.y = 0.0;
    my_pos.theta = 0.0;
    
    std::vector<pos3d_t> ref_traj;
    int total_length = 100;
    pos3d_t pos_tmp;
    for(int i = 0; i < total_length; i++){
        pos_tmp.x = i;
        pos_tmp.y = i;
        pos_tmp.theta = M_PI_2 / 2.0;
        pos_tmp.v = sqrt(2.0) / dt;
        ref_traj.push_back(pos_tmp);
    }

    std::vector<pos3d_t> todo_traj;

    std::vector<pos3d_t> obst_pos;
    int obst_num = 2;
    for(int i = 0; i < obst_num; i++){
        pos_tmp.x = i + 30;
        pos_tmp.y = 0;
        obst_pos.push_back(pos_tmp);
    }    
    
    std::cout << "planning!" << std::endl;
    bool succ_plan = my_planner.plan(my_pos, ref_traj, todo_traj, obst_pos);

    return 0;
}
