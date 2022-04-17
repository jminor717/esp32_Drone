FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/Common/*.*)

idf_component_register(SRCS ${app_sources}
                  INCLUDE_DIRS "../include/"
                  REQUIRES arduino)
