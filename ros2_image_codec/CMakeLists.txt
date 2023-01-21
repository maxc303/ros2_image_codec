cmake_minimum_required(VERSION 3.8)
project(ros2_image_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
)

add_library(image_codec lib_image_codec/ffmpeg_codec.cpp)
target_link_libraries(image_codec
  PkgConfig::LIBAV
  spdlog::spdlog
)
target_include_directories(image_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

add_executable(round_trip_test tests/round_trip_test.cpp)
target_link_libraries(round_trip_test
  spdlog::spdlog
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  PkgConfig::LIBAV
)

add_executable(encoder_node codec_nodes/encoder_node.cpp)
target_include_directories(encoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(encoder_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(encoder_node image_codec)

install(TARGETS encoder_node
  DESTINATION lib/${PROJECT_NAME})

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
  add_executable(ffmpeg_encoder_test lib_image_codec/ffmpeg_codec.test.cpp)
  target_link_libraries(ffmpeg_encoder_test PRIVATE Catch2::Catch2WithMain image_codec)
  # add_test(NAME ffmpeg_encoder_test COMMAND ffmpeg_encoder_test)
  ament_add_catch2_test(ffmpeg_encoder_test)

endif()

ament_package()
