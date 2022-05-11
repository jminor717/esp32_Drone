cmake_minimum_required(VERSION 3.5)

# set(EXTRA_COMPONENT_DIRS ${PROJECT_SOURCE_DIR}/Common/Submodules/RadioHead)
set(EXTRA_COMPONENT_DIRS "Submodules" "Common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/Common)
include_directories(${PROJECT_SOURCE_DIR}/Submodules)
# include_directories(${PROJECT_SOURCE_DIR}/Common/Submodules/RadioHead)

project(ESPDrone)


