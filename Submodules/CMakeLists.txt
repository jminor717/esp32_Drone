FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/Submodules/Arduino-PID-Library/*.* ${CMAKE_SOURCE_DIR}/Submodules/DShotRMT/src/*.* ${CMAKE_SOURCE_DIR}/Submodules/PID_AutoTune_v0/*.*)

idf_component_register(SRCS ${app_sources}
                  REQUIRES arduino)