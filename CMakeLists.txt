cmake_minimum_required(VERSION 3.5)

find_package( PythonInterp 3.8 REQUIRED )

add_custom_target(
 run ALL
 COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/buildscript_versioning.py ${PROJECT_SOURCE_DIR}
 BYPRODUCTS buildVersion.h
 COMMENT "running pre build python scripts"
)


# set(EXTRA_COMPONENT_DIRS ${PROJECT_SOURCE_DIR}/Common/Submodules/RadioHead)
set(EXTRA_COMPONENT_DIRS "Submodules" "Common" $ENV{IDF_PATH}/examples/common_components/led_strip)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/Common)
include_directories(${PROJECT_SOURCE_DIR}/Submodules)
# include_directories(${PROJECT_SOURCE_DIR}/Common/Submodules/RadioHead)

project(ESPDrone)


