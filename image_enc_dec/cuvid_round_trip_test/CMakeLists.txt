
project(cuvid_round_trip_test)

find_package(CUDA)
find_library(CUVID_LIB nvcuvid)
find_library(NVENCODEAPI_LIB nvidia-encode)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

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
message(INFO "NVCODEC PUBLIC INTERFACE DIR: ${NVCODEC_PUBLIC_INTERFACE_DIR}")


set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_50,code=\"sm_50,compute_50\")
if ( CMAKE_COMPILER_IS_GNUCC )
    if(NOT "${CUDA_NVCC_FLAGS}" MATCHES "-std=c\\+\\+11" )
        list(APPEND CUDA_NVCC_FLAGS -std=c++11)
    endif()
endif()

cuda_add_executable(${PROJECT_NAME} ${APP_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS}
 ${NVCODEC_PUBLIC_INTERFACE_DIR}
)

target_link_libraries(${PROJECT_NAME}  ${CUDA_CUDA_LIBRARY} ${CMAKE_DL_LIBS} ${NVENCODEAPI_LIB} ${CUVID_LIB})

target_link_libraries(${PROJECT_NAME}
  spdlog::spdlog
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)


