
cmake_minimum_required(VERSION 3.8)

project(cuvid_round_trip_test)

find_library(CUVID_LIB nvcuvid)
find_library(NVENCODEAPI_LIB nvidia-encode)

set(APP_SOURCES
 ${CMAKE_CURRENT_SOURCE_DIR}/cuvid_round_trip_test.cpp
)
#
# cuvid
#
if(NOT DEFINED ENV{VIDEO_CODEC_SDK_DIR})
  message(FATAL_ERROR "Nvidia Video codec sdk path(ENV{VIDEO_CODEC_SDK_DIR}) is not set.")
endif()


set(NVCODEC_PUBLIC_INTERFACE_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Interface)
set(NVCODEC_UTILS_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/Utils)
set(NV_CODEC_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/NvCodec)
set(NV_ENC_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/NvCodec/NvEncoder)
set(NV_DEC_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/NvCodec/NvDecoder)
set(NV_APPENC_COMMON_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/AppEncode/Common)
set(NV_APPDEC_COMMON_DIR  $ENV{VIDEO_CODEC_SDK_DIR}/Samples/AppDecode/Common)

set(NV_ENC_SOURCES
 ${NV_ENC_DIR}/NvEncoder.cpp
 ${NV_ENC_DIR}/NvEncoderCuda.cpp
)

set(NV_ENC_HDRS
 ${NV_ENC_DIR}/NvEncoder.h
 ${NV_ENC_DIR}/NvEncoderCuda.h
 ${NVCODEC_PUBLIC_INTERFACE_DIR}/nvEncodeAPI.h
 ${NVCODEC_UTILS_DIR}/NvCodecUtils.h
 ${NVCODEC_UTILS_DIR}/NvEncoderCLIOptions.h
)



source_group( "headers" FILES ${NV_ENC_HDRS} )
source_group( "sources" FILES  ${NV_ENC_SOURCES} ${NV_ENC_CUDA_UTILS})

find_package(CUDA)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_50,code=\"sm_50,compute_50\")
if ( CMAKE_COMPILER_IS_GNUCC )
    if(NOT "${CUDA_NVCC_FLAGS}" MATCHES "-std=c\\+\\+11" )
        list(APPEND CUDA_NVCC_FLAGS -std=c++11)
    endif()
endif()


cuda_add_executable(${PROJECT_NAME} ${APP_SOURCES} ${NV_ENC_SOURCES} ${NV_ENC_HDRS})

set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS}
 ${NVCODEC_PUBLIC_INTERFACE_DIR}
 ${NVCODEC_UTILS_DIR}
 ${NV_CODEC_DIR}
)

target_link_libraries(${PROJECT_NAME} ${CUDA_CUDA_LIBRARY} ${CMAKE_DL_LIBS} ${NVENCODEAPI_LIB} ${CUVID_LIB})
# target_link_libraries(${PROJECT_NAME}
#   spdlog::spdlog
#   ${OpenCV_LIBRARIES}
#   ${Boost_LIBRARIES}
# )


