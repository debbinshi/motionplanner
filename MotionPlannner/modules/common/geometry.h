#pragma once

#include <vector>
#include <string>

typedef struct {
    float x;
    float y;
    float z;
    float theta;
    float kappa;
    float v;// m/s
    float a;// m/s^2
    float da;// m/s^3
}pos3d_t;

typedef struct {
    int id;
    std::string type;
    std::vector<pos3d_t> pts;
}line3d_t;

typedef struct {
    int id;
    int lane_id;
    pos3d_t cpos;
    pos3d_t epos;
    std::vector<pos3d_t> corner_pos;
    bool is_occupied;
    bool is_checked;
    bool is_selected;
    double timestamp;
}lot_t;

typedef struct {
    int id;
    bool is_exit;
    pos3d_t pos;
    double timestamp;
}access_t;

typedef struct {
    int id;
    pos3d_t pos;
}topo_node_t;

typedef struct {
    int id;
    int lane_id;
    int from_id;
    int to_id;
}topo_edge_t;

typedef struct {
    int id;
    line3d_t cline;
    std::vector<line3d_t> blines;
}lane_t;

float get_2d_dis(pos3d_t p0, pos3d_t p1);

float get_theta(pos3d_t p0, pos3d_t p1);

float get_3d_dis(pos3d_t p0, pos3d_t p1);

uint32_t get_nearest_index(pos3d_t p0, std::vector<pos3d_t> pts, int from_idx, int to_idx);

float dot_product(pos3d_t p0, pos3d_t p1);

float cross_product(pos3d_t p0, pos3d_t p1);

pos3d_t get_foot_point(pos3d_t pt, pos3d_t line_start, pos3d_t line_end);

bool point_in_polygon(std::vector<pos3d_t> vertex, pos3d_t cur_pt);

int which_lane( pos3d_t pos, std::vector<lane_t> lane_list);

bool is_cross(pos3d_t a[2], pos3d_t b[2]);

bool lot_associate_lane(std::vector<lot_t> &lots, std::vector<lane_t> &lanes);
