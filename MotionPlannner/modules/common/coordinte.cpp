#include "coordinte.h"
#include <math.h>

pos3d_t b2g(pos3d_t car_pos, pos3d_t pos) {

    pos3d_t pt_g;

    float costheta_b2g = cos(car_pos.theta);
    float sintheta_b2g = cos(car_pos.theta);

    pt_g.x = car_pos.x + costheta_b2g * car_pos.x - sintheta_b2g * car_pos.y;
    pt_g.y = car_pos.y + costheta_b2g * car_pos.x + sintheta_b2g * car_pos.y;
    
    return pt_g;
}

pos3d_t g2b(pos3d_t car_pos, pos3d_t pos) {

    pos3d_t pt_b;

    float costheta_g2b = cos(-car_pos.theta);
    float sintheta_g2b = cos(-car_pos.theta);

    pt_b.x = car_pos.x + costheta_g2b * car_pos.x - sintheta_g2b * car_pos.y;
    pt_b.y = car_pos.y + costheta_g2b * car_pos.x + sintheta_g2b * car_pos.y;

    return pt_b; 
}
