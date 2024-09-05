cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.12.0)

if (NOT NM_FAMILY)
    set(NM_FAMILY "nm1801xx" CACHE PATH "NM build family")
endif()
if (NOT NM_TARGET)
    set(NM_TARGET "nm180100" CACHE PATH "NM build target")
endif()

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/targets/${NM_FAMILY}/${NM_TARGET}/build.cmake)

# Enable this option to re-build the rtos library
option(BUILD_RTOS "" OFF)

# Enable this option to re-build the hal library
option(BUILD_HAL "" OFF)

# Enable this option to re-build the LoRaWAN library
option(BUILD_LORAWAN "" OFF)

# Enable this option to re-build the ble library
option(BUILD_BLE "" OFF)

# Enable this option to re-build the LoRa mesh library
option(BUILD_LORA_MESH "" OFF)

project(nmsdk2)

add_definitions(-DNMI)

set(TARGET_NEWLIB_STUB_SRC ${PROJECT_SOURCE_DIR}/targets/${NM_FAMILY}/common/stub.c CACHE PATH "newlib stub file")
set(TARGET_STARTUP_SRC ${PROJECT_SOURCE_DIR}/targets/${NM_FAMILY}/${NM_TARGET}/startup_gcc.c CACHE PATH "startup file")
set(TARGET_LINK_SCRIPT ${PROJECT_SOURCE_DIR}/targets/${NM_FAMILY}/${NM_TARGET}/target.ld CACHE PATH "link script")
set(SDK_DIR ${PROJECT_SOURCE_DIR} CACHE PATH "SDK directory")

set(FAMILY_DIR targets/${NM_FAMILY})
set(FAMILY_COMMON_DIR targets/${NM_FAMILY}/common)
set(TARGET_DIR targets/${NM_FAMILY}/${NM_TARGET})

add_subdirectory(version)
add_subdirectory(${FAMILY_COMMON_DIR}/hal)
add_subdirectory(${TARGET_DIR})
add_subdirectory(${FAMILY_COMMON_DIR}/cmsis)

if (FEATURE_RAT_LORAWAN_ENABLE)
add_subdirectory(${TARGET_DIR}/comms/lorawan)
endif()

if (FEATURE_RAT_BLE_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/comms/ble)
endif()

if (FEATURE_RAT_LORA_MESH_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/comms/lora_direct)
add_subdirectory(${TARGET_DIR}/comms/lora_mesh)
endif()

if (FEATURE_RAT_LORA_DIRECT_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/comms/lora_direct)
endif()

if (FEATURE_TF_ENABLE)
add_subdirectory(${FAMILY_COMMON_DIR}/middleware/tflite-micro-2.6.5)
endif()
