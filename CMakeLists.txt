cmake_minimum_required(VERSION 3.13.1)

set(DTS_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/dts.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(camerapantilt)

target_sources(app PRIVATE src/main.c
                           src/grbl.c
                           src/visca.c
                           src/settings.c)
