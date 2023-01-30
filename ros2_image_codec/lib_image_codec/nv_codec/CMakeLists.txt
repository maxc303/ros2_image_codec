find_package(CUDA)
find_library(CUVID_LIB nvcuvid)
find_library(NVENCODEAPI_LIB nvidia-encode)

if(NOT DEFINED ENV{VIDEO_CODEC_SDK_DIR})
  message(FATAL_ERROR "Nvidia Video codec sdk path(ENV{VIDEO_CODEC_SDK_DIR}) is not set.")
else()
  message(STATUS "Using Nvidia Video codec sdk path: $ENV{VIDEO_CODEC_SDK_DIR}")
endif()

set(NVCODEC_PUBLIC_INTERFACE_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Interface)
set(NVCODEC_UTILS_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/Utils)
set(NV_CODEC_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/NvCodec)
set(NV_ENC_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/NvCodec/NvEncoder)
set(NV_DEC_DIR $ENV{VIDEO_CODEC_SDK_DIR}/Samples/NvCodec/NvDecoder)

set(NV_ENC_SOURCES
  ${NV_ENC_DIR}/NvEncoder.cpp
  ${NV_ENC_DIR}/NvEncoderCuda.cpp
)

set(NV_DEC_SOURCES
  ${NV_DEC_DIR}/NvDecoder.cpp
)

set(NV_ENC_HDRS
  ${NV_ENC_DIR}/NvEncoder.h
  ${NV_ENC_DIR}/NvEncoderCuda.h
  ${NVCODEC_PUBLIC_INTERFACE_DIR}/nvEncodeAPI.h
  ${NVCODEC_UTILS_DIR}/NvCodecUtils.h
  ${NVCODEC_UTILS_DIR}/NvEncoderCLIOptions.h
)

set(NV_DEC_HDRS
  ${NV_DEC_DIR}/NvDecoder.h
  ${NVCODEC_PUBLIC_INTERFACE_DIR}/cuviddec.h
  ${NVCODEC_PUBLIC_INTERFACE_DIR}/nvcuvid.h
  ${NVCODEC_UTILS_DIR}/NvCodecUtils.h
)

set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_50,code=\"sm_50,compute_50\")
if(CMAKE_COMPILER_IS_GNUCC)
    if(NOT "${CUDA_NVCC_FLAGS}" MATCHES "-std=c\\+\\+11")
        list(APPEND CUDA_NVCC_FLAGS -std=c++11)
    endif()
endif()

cuda_add_library(nv_codec ${NV_ENC_SOURCES}  ${NV_DEC_SOURCES}  ${NV_ENC_CUDA_UTILS} ${NV_ENC_HDRS}  ${NV_DEC_HDRS})
set_target_properties(nv_codec PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(nv_codec PRIVATE -Wall -Wextra)



# Add SYSTEM to suppress warning
target_include_directories(nv_codec SYSTEM PUBLIC ${CUDA_INCLUDE_DIRS}
  ${NVCODEC_PUBLIC_INTERFACE_DIR}
  ${NVCODEC_UTILS_DIR}
  ${NV_CODEC_DIR}
)

target_link_libraries(nv_codec  ${CUDA_CUDA_LIBRARY} ${CMAKE_DL_LIBS} ${NVENCODEAPI_LIB} ${CUVID_LIB})

# add_library(nv_codec nv_codec.cpp)
# target_link_libraries(nv_codec PUBLIC
#   nv_video_codec
# )

target_include_directories(nv_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
  $<INSTALL_INTERFACE:include>)


# if(BUILD_TESTING)
#   find_package(ament_cmake_catch2 REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)

#   # Disable uncrustify
#   set(ament_cmake_uncrustify_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()


#   find_package(Catch2 REQUIRED)
#   add_executable(nv_codec_test nv_codec.test.cpp)
#   target_link_libraries(nv_codec_test  nv_video_codec nv_codec Catch2::Catch2WithMain)
#   ament_add_catch2_test(nv_codec_test)

# endif()