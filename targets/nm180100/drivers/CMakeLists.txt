add_library(drv STATIC)

set(DRV_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set (DRV_INCLUDES
    ${DRV_DIR}/sx1262
    CACHE PATH "Driver includes"
)

target_include_directories(
    drv PRIVATE
    ${HAL_INCLUDES}
    ${DRV_INCLUDES}
)

target_sources(
    drv PRIVATE 
    ${DRV_DIR}/sx1262/sx1262.c
)

install (TARGETS drv DESTINATION ${PROJECT_SOURCE_DIR}/lib/${TARGET})