add_library(lora_direct STATIC)

set(LORA_DIRECT_DIR ${CMAKE_CURRENT_LIST_DIR})

set(
    LORA_DIRECT_INCLUDES
    ${LORA_DIRECT_DIR}
    CACHE PATH "LoRa Direct includes"
)

target_include_directories(
    lora_direct PRIVATE
    ${HAL_INCLUDES}
    ${LORA_DIRECT_INCLUDES}
)

target_sources(
    lora_direct PRIVATE 
    ${LORA_DIRECT_DIR}/lora_radio.c
    ${LORA_DIRECT_DIR}/sx1262.c
)