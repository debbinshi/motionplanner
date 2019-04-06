cmake_minimum_required(VERSION 2.8.3)
project(MotionPlannerNode)

add_compile_options(-std=c++11)
set(MPCPLANNER_INC_DIRS ${MP_ROOT_DIRS}/modules/mpc_planner)
set(COMMON_INC_DIRS ${MP_ROOT_DIRS}/modules/common)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${MPCPLANNER_INC_DIRS}
    ${COMMON_INC_DIRS}
)

add_executable(${PROJECT_NAME}
    MotionPlannerNode.cpp
)

target_link_libraries(${PROJECT_NAME}
    common
    mpc_planner
    ${catkin_LIBRARIES}
)
