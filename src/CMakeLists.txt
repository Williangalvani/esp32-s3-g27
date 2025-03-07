# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                       REQUIRES usb esp_tinyusb
                       INCLUDE_DIRS 
                         "../managed_components/espressif__esp_tinyusb/include"
                         "../managed_components/espressif__tinyusb/src")