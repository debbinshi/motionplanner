cmake_minimum_required (VERSION 2.6)
project(MotionPlanner)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Debug")
message("CMAKE_BUILD_TYPE = " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g2 -ggdb")
message("CMAKE_CXX_FLAGS_DEBUG" ${CMAKE_CXX_FLAGS_DEBUG})
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g2 -Wall")
message("CMAKE_CXX_FLAGS_RELEASE" ${CMAKE_CXX_FLAGS_RELEASE})
set(CMAKE_SYS_ARCH ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR})
message("CMAKE_SYS_ARCH" ${CMAKE_SYS_ARCH})

set(MP_ROOT_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
message("MotionPlanner = " ${MP_ROOT_DIRS})

add_subdirectory(modules)
add_subdirectory(examples)
