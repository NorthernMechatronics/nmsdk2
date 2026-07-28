cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.13.0)

if (NM_TARGET STREQUAL "nm180100")
    set(NM_FAMILY "nm1801xx" CACHE PATH "nm1801xx family")
    set(CMSIS_SYSTEM_SRC "system_apollo3.c" CACHE PATH "Apollo3")
elseif (NM_TARGET STREQUAL "nm180110")
    set(NM_FAMILY "nm1801xx" CACHE PATH "nm1801xx family")
    set(CMSIS_SYSTEM_SRC "system_apollo3p.c" CACHE PATH "Apollo3 Plus")
elseif (NM_TARGET STREQUAL "apollo510")
    set(NM_FAMILY "cortex-m55" CACHE PATH "cortex-m55 family")
    set(CMSIS_SYSTEM_SRC "system_apollo510.c" CACHE PATH "Apollo510")
    set(FEATURE_RAT_BLE_ENABLE OFF CACHE BOOL "" FORCE)
    set(FEATURE_RAT_LORAWAN_ENABLE OFF CACHE BOOL "" FORCE)
    set(FEATURE_RAT_LORA_MESH_ENABLE OFF CACHE BOOL "" FORCE)
    set(FEATURE_RAT_LORADIRECT_ENABLE OFF CACHE BOOL "" FORCE)
else()
    message(FATAL_ERROR "Unknown NM_TARGET: ${NM_TARGET}")
endif()

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/targets/${NM_FAMILY}/${NM_TARGET}/build.cmake)

# Enable this option to re-build the rtos library
option(BUILD_RTOS "" OFF)

# Enable this option to re-build the hal library
option(BUILD_HAL "" OFF)

# Enable this option to re-build the LoRaWAN library
option(BUILD_LORAWAN "" OFF)

# Enable this option to re-build the LoRa Basic Modem library
option(BUILD_LBM "" OFF)

# Enable this option to re-build the ble library
option(BUILD_BLE "" OFF)

# Enable this option to re-build the LoRa mesh library
option(BUILD_LORA_MESH "" OFF)

# Enable this option to re-build the Tensorflow Lite Micro library
option(BUILD_TFLM "" OFF)

project(nmsdk2)

add_definitions(-DNMI)

set(TARGET_NEWLIB_STUB_SRC ${PROJECT_SOURCE_DIR}/targets/${NM_FAMILY}/common/stub.c CACHE PATH "newlib stub file")

set(TARGET_STARTUP_SRC
    ${PROJECT_SOURCE_DIR}/targets/${NM_FAMILY}/${NM_TARGET}/startup_gcc.c
    ${PROJECT_SOURCE_DIR}/hal/ambiq/CMSIS/AmbiqMicro/Source/${CMSIS_SYSTEM_SRC}
    CACHE PATH "startup file")

set(TARGET_LINK_SCRIPT ${PROJECT_SOURCE_DIR}/targets/${NM_FAMILY}/${NM_TARGET}/target.ld CACHE PATH "link script")
set(SDK_DIR ${PROJECT_SOURCE_DIR} CACHE PATH "SDK directory")

set(FAMILY_DIR targets/${NM_FAMILY})
set(FAMILY_COMMON_DIR targets/${NM_FAMILY}/common)
set(TARGET_DIR targets/${NM_FAMILY}/${NM_TARGET})

add_subdirectory(version)
add_subdirectory(${FAMILY_COMMON_DIR}/hal)
add_subdirectory(${TARGET_DIR})
add_subdirectory(${FAMILY_COMMON_DIR}/cmsis)

add_subdirectory(doc EXCLUDE_FROM_ALL)

if (FEATURE_RAT_LORAWAN_ENABLE)
add_subdirectory(${TARGET_DIR}/comms/lorawan)
# add_subdirectory(${TARGET_DIR}/comms/SWL2001)
endif()

if (FEATURE_RAT_BLE_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/comms/ble)
endif()

if (FEATURE_RAT_LORA_MESH_ENABLE)
add_subdirectory(${TARGET_DIR}/comms/lora_mesh)
endif()

if (FEATURE_RAT_LORA_DIRECT_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/comms/lora_direct)
endif()

if (FEATURE_TFLM_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/middleware/tflm)
endif()
