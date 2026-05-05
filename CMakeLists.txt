cmake_minimum_required(VERSION 3.15)

# Common source files
set(srcs
  src/dsp/dsps_dotprod_f32_ansi.c
  src/resample/art_biquad.cpp
  src/resample/art_resampler.cpp
  src/resample/resampler.cpp
  src/quantization_utils.cpp
  src/memory_utils.cpp
  src/gain.cpp
  )

if(ESP_PLATFORM)
  # Add ESP32-specific assembly optimizations
  list(APPEND srcs
    src/dsp/dsps_dotprod_f32_ae32.S
    src/dsp/dsps_dotprod_f32_aes3.S
    src/dsp/dsps_dotprod_f32_m_ae32.S
  )

  # Build as an ESP-IDF component
  idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
  )

  # Apply -O2 optimization for ESP-IDF builds
  target_compile_options(${COMPONENT_LIB} PRIVATE -O2)

  return()
endif()

project(esp-audio-libs VERSION 3.0.0)

# Standalone build configuration
add_library(esp-audio-libs STATIC ${srcs})

target_include_directories(esp-audio-libs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++ standard
target_compile_features(esp-audio-libs PUBLIC cxx_std_11)

# Add compile options
target_compile_options(esp-audio-libs PRIVATE -O2)

# Installation rules
install(TARGETS esp-audio-libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
