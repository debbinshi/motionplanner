cmake_minimum_required (VERSION 2.6)
project(common)

aux_source_directory(./ DIR_SRCS)

add_library(${PROJECT_NAME}
    ${DIR_SRCS}
)
