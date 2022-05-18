FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/Submodules/FastPID/src/*.* ${CMAKE_SOURCE_DIR}/Submodules/DShotRMT/src/*.*)

idf_component_register(SRCS ${app_sources}
                  REQUIRES arduino)