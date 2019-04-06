#include "geometry.h"
#include <math.h>
#include <stdint.h>
#include <limits>

float get_2d_dis(pos3d_t p0, pos3d_t p1) {
    return sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y)); 
}

float get_theta(pos3d_t p0, pos3d_t p1) {
    return atan2(p1.y - p0.y, p1.x - p0.x);
}

float get_3d_dis(pos3d_t p0, pos3d_t p1) {
    return sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y) + (p1.z - p0.z) * (p1.z - p0.z)); 
}

uint32_t get_nearest_index(pos3d_t p0, std::vector<pos3d_t> pts, int from_idx, int to_idx) {
    int index = -1;
    float min_dis = (std::numeric_limits<float>::max)();

    for(int i = from_idx; i < to_idx; i++) {
        float dis = get_2d_dis(p0, pts[i]);
        float theta_err = pts[i].theta - p0.theta;

        if(theta_err > M_PI) theta_err -= 2 * M_PI;
        if(theta_err > -M_PI) theta_err += 2 * M_PI;
        if(theta_err > M_PI) theta_err -= 2 * M_PI;
        if(theta_err < -M_PI) theta_err += 2 * M_PI;

        if ((dis < min_dis) && (fabs(theta_err) < M_PI_2)) {
            min_dis = dis;
            index = i;
        }
    } 

    return index;
}

float dot_product(pos3d_t p0, pos3d_t p1) {
    return p0.x * p1.x + p0.y * p1.y;
}

float cross_product(pos3d_t p0, pos3d_t p1) {
    return (p0.x * p1.y - p1.x * p0.y);
}

pos3d_t get_foot_point(pos3d_t pt, pos3d_t line_start, pos3d_t line_end) {
    pos3d_t ret_pt;

    float dx = line_start.x - line_end.x;
    float dy = line_start.y - line_end.y;
    if(fabs(dx) < 0.f && fabs(dy) < 0.f) {
        ret_pt = line_start;
        return ret_pt;
    }

    float u = (pt.x - line_start.x) * (line_start.x - line_end.x) + (pt.y - line_start.y) * (line_start.y - line_end.y);
    u = u / (dx * dx + dy * dy);

    ret_pt.x = line_start.x + u * dx;
    ret_pt.y = line_start.y + u * dy;

    return ret_pt;
}

bool point_in_polygon(std::vector<pos3d_t> vertex, pos3d_t cur_pt) {

    uint32_t i, j = vertex.size() - 1;
    bool is_odd = false;
    float x = cur_pt.x;
    float y = cur_pt.y;

    for(i = 0; i < vertex.size(); i++) {
        if(((vertex[i].y < y && vertex[j].y >= y)
            || (vertex[j].y < y && vertex[i].y >= y))
            && (vertex[i].x <= x && vertex[j].x <= x)) {

            //x-x1 / x2-x1 = y-y1 / y2-y1;
            //x = y-y1 / y2-y1 * x2-x1 + x1;
            if(vertex[i].x + (y - vertex[i].y) / (vertex[j].y - vertex[i].y) * (vertex[j].x - vertex[i].x) < x)
                is_odd = !is_odd;
        }

        j = i;
    }

    return is_odd;
}

int which_lane( pos3d_t pos, std::vector<lane_t> lane_list) {
    int lane_id = 0;
    float min_dis = 100;

    for(uint32_t i = 0; i < lane_list.size(); i++) {
        pos3d_t start_pt = lane_list[i].cline.pts.front();
        pos3d_t end_pt = lane_list[i].cline.pts.back();

        float l0 = get_2d_dis(start_pt, end_pt);
        float l1 = get_2d_dis(start_pt, pos);
        float l2 = get_2d_dis(end_pt, pos);
        float p = 0.5 * (l0 + l1 + l2);
        float S = sqrt(p * (p - l0) * (p - l1) * (p - l2));

        float dis = 2 * S / l0;
        if(min_dis > dis) {
            lane_id = lane_list[i].id;
            min_dis = dis;
        }
    }

    return lane_id;
}

bool is_cross(pos3d_t a[2], pos3d_t b[2]) {

    if(
        std::min(a[1].x, a[0].x) <= std::max(b[0].x, b[1].x) &&
        std::min(b[1].x, b[0].x) <= std::max(a[0].x, a[1].x) &&
        std::min(a[1].y, a[0].y) <= std::max(b[0].y, b[1].y) &&
        std::min(b[1].y, b[0].y) <= std::max(a[0].y, a[1].x)
    ) {

        pos3d_t a0b0, b1b0, a1a0, b1a0;

        a0b0.x = a[0].x - b[0].x;
        a0b0.y = a[0].y - b[0].y;

        b1b0.x = b[1].x - b[0].x;
        b1b0.y = b[1].y - b[0].y;

        a1a0.x = a[1].x - a[0].x;
        a1a0.y = a[1].y - a[0].y;

        b1a0.x = b[1].x - a[0].x;
        b1a0.y = b[1].y - a[0].y;

        //(A0-B0) x (B1-B0) * (B1-B0) x (A1-A0) >= 0
        //(B0-A0) x (A1-A0) * (A1-A0) x (B1-A0) >= 0
        if(cross_product(a0b0, b1b0) * cross_product(b1b0, a1a0) >= 0) {

            a0b0.y = -a0b0.y;
            a0b0.x = -a0b0.x;

            if(cross_product(a0b0, a1a0) * cross_product(a1a0, b1a0) >= 0)
                return true;
            else
                return false;
        } else {
            return false;
        }
    }

    return false;
}

bool lot_associate_lane(std::vector<lot_t> &lots, std::vector<lane_t> &lanes) {

    for(uint32_t i = 0; i < lots.size(); i++) {
        float min_dis_lane = 0;
        float min_dis = 1000;
        for(uint32_t j = 0; j < lanes.size(); j++) {
            float tmp_dis = 0;

            for(uint32_t k = 0; k < lanes[j].cline.pts.size() - 1; k++) {
                pos3d_t foot_point;
                foot_point = get_foot_point( lots[i].epos,
                                                lanes[j].cline.pts[k],
                                                lanes[j].cline.pts[k + 1]);

                float ang_0 = get_theta(lots[i].cpos, lots[i].epos);
                float ang_1 = get_theta(lots[i].epos, foot_point);
                float delta_ang = abs(ang_0 - ang_1);

                //????约束有问题
                if(delta_ang > 0.01)
                    continue;

                pos3d_t a,b,c,d;
                a.x = lanes[j].cline.pts[k].x;
                a.y = lanes[j].cline.pts[k].y;
                b.x = lanes[j].cline.pts[k + 1].x;
                b.y = lanes[j].cline.pts[k].y;
                c.x = lanes[j].cline.pts[k + 1].x;
                c.y = lanes[j].cline.pts[k + 1].y;
                d.x = lanes[j].cline.pts[k].x;
                d.y = lanes[j].cline.pts[k + 1].y;

                std::vector<pos3d_t> tmp_pt_list;
                tmp_pt_list.push_back(a);
                tmp_pt_list.push_back(b);
                tmp_pt_list.push_back(c);
                tmp_pt_list.push_back(d);

                bool is_in = point_in_polygon(tmp_pt_list, foot_point);
                if(!is_in) {
                    continue;
                }

                tmp_dis = get_2d_dis(foot_point, lots[i].epos);
                if(min_dis > tmp_dis) {
                    min_dis = tmp_dis;
                    min_dis_lane = lanes[j].id;
                }
            }
        }

        lots[i].lane_id = min_dis_lane;
    }

    return true;
}
