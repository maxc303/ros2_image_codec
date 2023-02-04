

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
  add_executable(codec_tests codec_tests.cpp)
  target_link_libraries(codec_tests Catch2::Catch2WithMain nv_codec ffmpeg_codec )
  ament_add_catch2_test(codec_tests)

endif()