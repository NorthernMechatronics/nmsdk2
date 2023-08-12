add_library(drivers STATIC)

set(DRIVERS_DIR ${CMAKE_CURRENT_LIST_DIR})

set(
    DRIVERS_INCLUDES
    ${DRIVERS_DIR}/lora
    CACHE PATH "DRIVERS includes"
)

target_include_directories(
    drivers PRIVATE
    ${HAL_INCLUDES}
    ${DRIVERS_INCLUDES}
)

target_sources(
    drivers PRIVATE 
    ${DRIVERS_DIR}/lora/lora_radio.c
    ${DRIVERS_DIR}/lora/sx1262.c
)

install (TARGETS drivers DESTINATION ${PROJECT_SOURCE_DIR}/lib/${TARGET})