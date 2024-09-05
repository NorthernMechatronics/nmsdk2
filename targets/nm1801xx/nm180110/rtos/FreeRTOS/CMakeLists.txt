
set(RTOS_DIR ${PROJECT_SOURCE_DIR}/rtos/FreeRTOS)
set(RTOS_TARGET_DIR ${CMAKE_CURRENT_LIST_DIR})

set (RTOS_INCLUDES
    ${RTOS_DIR}/kernel/include
    ${RTOS_DIR}/cli
    ${RTOS_TARGET_DIR}
    ${RTOS_TARGET_DIR}/portable
    CACHE PATH "RTOS includes"
)

if (BUILD_RTOS)

message("RTOS development enabled")

add_library(rtos STATIC)

target_include_directories(
    rtos PRIVATE
    ${HAL_INCLUDES}
    ${RTOS_INCLUDES}
)

target_sources(
    rtos PRIVATE 
    ${RTOS_DIR}/kernel/croutine.c
    ${RTOS_DIR}/kernel/event_groups.c
    ${RTOS_DIR}/kernel/list.c
    ${RTOS_DIR}/kernel/queue.c
    ${RTOS_DIR}/kernel/stream_buffer.c
    ${RTOS_DIR}/kernel/tasks.c
    ${RTOS_DIR}/kernel/timers.c
    ${RTOS_DIR}/cli/FreeRTOS_CLI.c

    ${RTOS_TARGET_DIR}/portable/heap_4.c
    ${RTOS_TARGET_DIR}/portable/port.c
)

install(
    TARGETS rtos
    DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

set(RTOS_LIBS
    $<TARGET_FILE:rtos>
    CACHE PATH "RTOS libs"
)

else(BUILD_RTOS)

set(RTOS_LIBS
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/librtos.a
    CACHE PATH "RTOS libs"
)

endif(BUILD_RTOS)