cmake_minimum_required (VERSION 2.6)
project(mpc_planner)

set(COMMON_INC_DIRS ${MP_ROOT_DIRS}/modules/common)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(
    ${COMMON_INC_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    mpc_planner.cpp
)
target_link_libraries(${PROJECT_NAME}
    common
    ${CERES_LIBRARIES}
)
