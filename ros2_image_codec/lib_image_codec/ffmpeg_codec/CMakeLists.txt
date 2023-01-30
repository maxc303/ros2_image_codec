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

  if(BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  # Disable uncrustify
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()


  find_package(Catch2 REQUIRED)
  add_executable(ffmpeg_encoder_test ffmpeg_codec.test.cpp)
  target_link_libraries(ffmpeg_encoder_test PRIVATE Catch2::Catch2WithMain image_codec)
  # add_test(NAME ffmpeg_encoder_test COMMAND ffmpeg_encoder_test)
  ament_add_catch2_test(ffmpeg_encoder_test)

endif()