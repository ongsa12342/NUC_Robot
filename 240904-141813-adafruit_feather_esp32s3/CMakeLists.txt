cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(240904-141813-adafruit_feather_esp32s3)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
set(IDF_TARGET esp32s3)
set(COMPONENTS arduino)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components/arduino)

# Add your component directory
set(COMPONENTS esp_pulse_cnt)

# Include the component directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/components/esp-idf/components/esp_driver_pcnt/include)

# Add source files
set(pulse_cnt_srcs ${CMAKE_CURRENT_SOURCE_DIR}/ccomponents/esp-idf/components/esp_driver_pcnt/src/pulse_cnt.c)

# Add the source files to the build
target_sources(${COMPONENT_TARGET} PRIVATE ${pulse_cnt_srcs})

idf_component_register(
    # SRCS "mobile-control.cpp"
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino
)

