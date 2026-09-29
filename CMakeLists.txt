# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# esp_lcd is not used by this project. It gets built anyway by default (IDF
# builds every discovered component), and its rgb/esp_lcd_panel_rgb.c triggers
# a GCC ICE (segfault in RTL pass "ira") on this toolchain for the S3 target.
set(EXCLUDE_COMPONENTS "esp_lcd")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESPelemetry)
