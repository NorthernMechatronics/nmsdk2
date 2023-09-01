cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.12.0)

set(TARGET "nm180100" CACHE STRING "Build target")
set_property(
    CACHE TARGET
    PROPERTY STRINGS
    "nm180100"
)

project(nmsdk2)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_DEBUG_POSTFIX -dbg)
set(CMAKE_RELEASE_POSTFIX -rel)

set(TARGET_SRC ${PROJECT_SOURCE_DIR}/targets/${TARGET}/startup_gcc.c CACHE PATH "startup file")
set(TARGET_LINK_SCRIPT ${PROJECT_SOURCE_DIR}/targets/${TARGET}/AMA3B1KK.ld CACHE PATH "link script")

add_subdirectory(targets/${TARGET}/cmsis)
add_subdirectory(targets/${TARGET}/hal)
add_subdirectory(targets/${TARGET}/rtos/FreeRTOS)
add_subdirectory(targets/${TARGET}/drivers)

if (RAT_LORAWAN_ENABLE)
add_subdirectory(targets/${TARGET}/comms/lorawan)
endif()

if (RAT_BLE_ENABLE)
add_subdirectory(targets/${TARGET}/comms/ble)
endif()

if (TF_ENABLE)
add_subdirectory(targets/${TARGET}/middleware/tflite-micro-2.6.5)
endif()
