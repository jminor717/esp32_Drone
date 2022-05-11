# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# file(GLOB_RECURSE srcs
#   "*.c"
# )

# idf_component_register(SRCS ${srcs}
#                     INCLUDE_DIRS "../include/")

# idf_component_get_property( FREERTOS_ORIG_INCLUDE_PATH freertos ORIG_INCLUDE_PATH)
# target_include_directories(${COMPONENT_TARGET} PUBLIC
#  "${FREERTOS_ORIG_INCLUDE_PATH}"
# )

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources}
                        INCLUDE_DIRS "../include/"
                        REQUIRES Common Submodules esp_adc_cal spi_flash nvs_flash arduino)

# idf_component_register(SRCS ${srcs}
#                     INCLUDE_DIRS "../Common/")