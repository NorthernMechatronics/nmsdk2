
set(HAL_DIR ${PROJECT_SOURCE_DIR}/hal/ambiq)
if ("${NM_TARGET}" STREQUAL "nm180100")
set(HAL_APOLLO_DIR ${HAL_DIR}/mcu/apollo3)
elseif ("${NM_TARGET}" STREQUAL "nm180110")
set(HAL_APOLLO_DIR ${HAL_DIR}/mcu/apollo3p)
else()
message(FATAL_ERROR "Target not supported")
endif()

set (HAL_INCLUDES
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS_6/CMSIS/Core/Include
    ${HAL_DIR}/CMSIS/AmbiqMicro/Include
    ${HAL_DIR}/mcu
    ${HAL_APOLLO_DIR}
    ${HAL_APOLLO_DIR}/hal
    ${HAL_APOLLO_DIR}/regs
    ${HAL_DIR}/utils
    CACHE PATH "HAL includes"
)

if (BUILD_HAL)

message("HAL development enabled")

add_library(hal STATIC)

target_include_directories(
    hal PRIVATE
    ${HAL_INCLUDES}
)

target_sources(
    hal PRIVATE
    ${HAL_APOLLO_DIR}/hal/am_hal_adc.c
    ${HAL_APOLLO_DIR}/hal/am_hal_ble.c
    ${HAL_APOLLO_DIR}/hal/am_hal_ble_patch_b0.c
    ${HAL_APOLLO_DIR}/hal/am_hal_burst.c
    ${HAL_APOLLO_DIR}/hal/am_hal_cachectrl.c
    ${HAL_APOLLO_DIR}/hal/am_hal_clkgen.c
    ${HAL_APOLLO_DIR}/hal/am_hal_cmdq.c
    ${HAL_APOLLO_DIR}/hal/am_hal_ctimer.c
    ${HAL_APOLLO_DIR}/hal/am_hal_debug.c
    ${HAL_APOLLO_DIR}/hal/am_hal_entropy.c
    ${HAL_APOLLO_DIR}/hal/am_hal_flash.c
    ${HAL_APOLLO_DIR}/hal/am_hal_global.c
    ${HAL_APOLLO_DIR}/hal/am_hal_gpio.c
    ${HAL_APOLLO_DIR}/hal/am_hal_interrupt.c
    ${HAL_APOLLO_DIR}/hal/am_hal_iom.c
    ${HAL_APOLLO_DIR}/hal/am_hal_ios.c
    ${HAL_APOLLO_DIR}/hal/am_hal_itm.c
    ${HAL_APOLLO_DIR}/hal/am_hal_mcuctrl.c
    ${HAL_APOLLO_DIR}/hal/am_hal_mspi.c
    ${HAL_APOLLO_DIR}/hal/am_hal_pdm.c
    ${HAL_APOLLO_DIR}/hal/am_hal_pwrctrl.c
    ${HAL_APOLLO_DIR}/hal/am_hal_queue.c
    ${HAL_APOLLO_DIR}/hal/am_hal_reset.c
    ${HAL_APOLLO_DIR}/hal/am_hal_rtc.c
    ${HAL_APOLLO_DIR}/hal/am_hal_scard.c
    ${HAL_APOLLO_DIR}/hal/am_hal_secure_ota.c
    ${HAL_APOLLO_DIR}/hal/am_hal_security.c
    ${HAL_APOLLO_DIR}/hal/am_hal_stimer.c
    ${HAL_APOLLO_DIR}/hal/am_hal_sysctrl.c
    ${HAL_APOLLO_DIR}/hal/am_hal_systick.c
    ${HAL_APOLLO_DIR}/hal/am_hal_tpiu.c
    ${HAL_APOLLO_DIR}/hal/am_hal_uart.c
    ${HAL_APOLLO_DIR}/hal/am_hal_wdt.c

    ${HAL_DIR}/utils/am_util_ble.c
    ${HAL_DIR}/utils/am_util_debug.c
    ${HAL_DIR}/utils/am_util_delay.c
    ${HAL_DIR}/utils/am_util_faultisr.c
    ${HAL_DIR}/utils/am_util_id.c
    # ${HAL_DIR}/utils/am_util_regdump.c
    ${HAL_DIR}/utils/am_util_stdio.c
    ${HAL_DIR}/utils/am_util_string.c
    ${HAL_DIR}/utils/am_util_time.c
)

if ("${NM_TARGET}" STREQUAL "nm180100")
target_sources(
    hal PRIVATE
    ${HAL_APOLLO_DIR}/hal/am_hal_ble_patch.c
)
endif()

install(
    TARGETS hal
    DESTINATION ${PROJECT_SOURCE_DIR}/${TARGET_DIR}/hal/lib
)

set(HAL_LIBS
    $<TARGET_FILE:hal>
    CACHE PATH "HAL libs"
)

else(BUILD_HAL)

set(HAL_LIBS
    ${PROJECT_SOURCE_DIR}/${TARGET_DIR}/hal/lib/libhal.a
    CACHE PATH "HAL libs"
)

endif(BUILD_HAL)