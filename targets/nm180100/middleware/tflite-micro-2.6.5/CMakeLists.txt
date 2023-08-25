add_library(tflm STATIC)

set(TFLM_DIR ${PROJECT_SOURCE_DIR}/middleware/tflite-micro-2.6.5)

set (TFLM_INCLUDES
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS_5/CMSIS/Core/Include
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS-DSP/Include
    ${PROJECT_SOURCE_DIR}/cmsis/CMSIS-NN/Include
    ${TFLM_DIR}
    ${TFLM_DIR}/third_party/flatbuffers/include
    ${TFLM_DIR}/third_party/gemmlowp
    ${TFLM_DIR}/third_party/ruy
    CACHE PATH "TFLM includes"
)

target_compile_definitions(
    tflm PRIVATE
	-DGEMMLOWP_ALLOW_SLOW_SCALAR_FALLBACK
    -DTF_LITE_STATIC_MEMORY
    -DTF_LITE_MCU_DEBUG_LOG
    -DARM_MATH_CM4
    -DCMSIS_NN
)

target_include_directories(
    tflm PRIVATE
    ${TFLM_INCLUDES}
    ${HAL_INCLUDES}
)

target_sources(
    tflm PRIVATE 
    debug_log.cc 
    micro_time.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/all_ops_resolver.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/memory_helpers.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_allocator.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_error_reporter.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_graph.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_interpreter.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_profiler.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_string.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/micro_utils.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/mock_micro_graph.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/recording_micro_allocator.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/recording_simple_memory_allocator.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/simple_memory_allocator.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/system_setup.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/test_helpers.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/memory_planner/greedy_memory_planner.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/memory_planner/linear_memory_planner.cc 
    ${TFLM_DIR}/tensorflow/lite/c/common.c 
    ${TFLM_DIR}/tensorflow/lite/core/api/error_reporter.cc 
    ${TFLM_DIR}/tensorflow/lite/core/api/flatbuffer_conversions.cc 
    ${TFLM_DIR}/tensorflow/lite/core/api/op_resolver.cc 
    ${TFLM_DIR}/tensorflow/lite/core/api/tensor_utils.cc 
    ${TFLM_DIR}/tensorflow/lite/kernels/internal/quantization_util.cc 
    ${TFLM_DIR}/tensorflow/lite/kernels/kernel_util.cc 
    ${TFLM_DIR}/tensorflow/lite/schema/schema_utils.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/activations.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/add_n.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/arg_min_max.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/batch_to_space_nd.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cast.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/ceil.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/circular_buffer.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/comparisons.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/concatenation.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/conv_common.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cumsum.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/depth_to_space.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/depthwise_conv_common.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/dequantize.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/detection_postprocess.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/elementwise.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/elu.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/ethosu.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/exp.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/expand_dims.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/fill.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/floor.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/floor_div.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/floor_mod.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/fully_connected_common.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/gather.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/gather_nd.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/hard_swish.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/if.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/kernel_runner.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/kernel_util.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/l2norm.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/l2_pool_2d.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/leaky_relu.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/logical.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/logistic.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/log_softmax.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/maximum_minimum.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/neg.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/pack.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/pad.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/prelu.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/quantize.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/quantize_common.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/reduce.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/reshape.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/resize_bilinear.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/resize_nearest_neighbor.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/round.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/shape.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/softmax_common.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/space_to_batch_nd.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/split.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/split_v.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/squeeze.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/strided_slice.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/sub.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/svdf_common.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/tanh.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/transpose.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/transpose_conv.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/unpack.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/zeros_like.cc 

    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/add.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/conv.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/depthwise_conv.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/fully_connected.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/mul.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/pooling.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/softmax.cc 
    # ${TFLM_DIR}/tensorflow/lite/micro/kernels/svdf.cc 

    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/add.cc
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/conv.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/depthwise_conv.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/fully_connected.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/mul.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/pooling.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/softmax.cc 
    ${TFLM_DIR}/tensorflow/lite/micro/kernels/cmsis_nn/svdf.cc 
)

install (TARGETS tflm DESTINATION ${PROJECT_SOURCE_DIR}/lib/${TARGET})