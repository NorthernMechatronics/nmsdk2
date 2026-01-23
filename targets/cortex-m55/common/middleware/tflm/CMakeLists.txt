set(TFLM_DIR ${PROJECT_SOURCE_DIR}/middleware/tflm)

set (TFLM_INCLUDES
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS_6
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS_6/CMSIS/Core/Include
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS-NN
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS-NN/Include
    ${TFLM_DIR}
    ${TFLM_DIR}/third_party/gemmlowp
    ${TFLM_DIR}/third_party/flatbuffers/include
    ${TFLM_DIR}/third_party/kissfft
    ${TFLM_DIR}/third_party/kissfft/tools
    ${TFLM_DIR}/third_party/ruy
    CACHE PATH "TFLM includes"
)

if (BUILD_TFLM)

message("Tensorflow Lite Micro development enabled")

add_library(tflm STATIC)

set(TFLM_C_FLAGS
    -fno-unwind-tables
    -funsigned-char
    CACHE PATH "TFLM C flags"
)

set(TFLM_CXX_FLAGS
    -fno-unwind-tables
    -funsigned-char
    -fno-threadsafe-statics
    CACHE PATH "TFLM C++ flags"
)

target_compile_definitions(
    tflm PRIVATE
    -DTF_LITE_STATIC_MEMORY
    -DTF_LITE_MCU_DEBUG_LOG
    -DCMSIS_NN
)

target_compile_options(
    tflm PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${TFLM_C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${TFLM_CXX_FLAGS}>
)

target_include_directories(
    tflm PRIVATE
    ${TFLM_INCLUDES}
    ${HAL_INCLUDES}
)

file(GLOB_RECURSE TFLM_SRC
    LIST_DIRECTORIES false
    "${TFLM_DIR}/tensorflow/*.cc"
    "${TFLM_DIR}/tensorflow/*.c"
    "${TFLM_DIR}/signal/*.cc"
    "${TFLM_DIR}/signal/*.c"
)

target_sources(
    tflm PRIVATE 
    debug_log.cc 
    micro_time.cc
    ${TFLM_SRC}
)

install (TARGETS tflm DESTINATION ${PROJECT_SOURCE_DIR}/${TARGET_DIR}/middleware/tflm/lib)

set(TFLM_LIBS
    $<TARGET_FILE:tflm>
    CACHE PATH "TFLM libs"
)

else(BUILD_TFLM)

set(TFLM_LIBS
    ${PROJECT_SOURCE_DIR}/${TARGET_DIR}/middleware/tflm/lib/libtflm.a
    CACHE PATH "TFLM libs"
)

endif(BUILD_TFLM)