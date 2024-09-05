set(NM_SDK_VERSION_H ${CMAKE_CURRENT_LIST_DIR}/nm_sdk_version.h CACHE PATH "")
set(NM_SDK_VERSION_C ${CMAKE_CURRENT_LIST_DIR}/nm_sdk_version.c CACHE PATH "")

add_library(
    nm_sdk_version
    STATIC
)

set (NM_SDK_VERSION_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}
    CACHE PATH "SDK version includes"
)

target_include_directories(
    nm_sdk_version
    PRIVATE
    ${NM_SDK_VERSION_INCLUDES}
)

target_sources(
    nm_sdk_version
    PRIVATE
    ${NM_SDK_VERSION_C}
)
