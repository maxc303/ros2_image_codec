find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
)

add_library(ffmpeg_codec ffmpeg_codec.cpp)
target_link_libraries(ffmpeg_codec
  PkgConfig::LIBAV
)

target_include_directories(ffmpeg_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
  $<INSTALL_INTERFACE:include>)

