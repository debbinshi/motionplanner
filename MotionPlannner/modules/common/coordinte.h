#pragma once

#include "geometry.h"

/*
*car body coordinte
*
*            X 
*            .
*       0    .    1 
*        .........
*        .       .
*        .       .
*        .       .
*        .       .
*  Y......       .
*        .........
*       3         2
*
*
*location coodrinate
*          
*          y 
*          .
*          .
*        .........
*        .       ....x, theta = 0 degree
*        .........
*
*  .................map_x
*/


pos3d_t b2g(pos3d_t car_pos, pos3d_t pos);

pos3d_t g2b(pos3d_t car_pos, pos3d_t pos);
