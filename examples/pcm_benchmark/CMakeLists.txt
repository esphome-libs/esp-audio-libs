# esp-audio-libs PCM convert / mixer benchmark example

cmake_minimum_required(VERSION 3.16)

# Include the esp-audio-libs component from the repository root (two levels up).
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pcm_benchmark)
