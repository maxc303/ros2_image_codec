cmake_minimum_required(VERSION 3.8)
project(image_enc_dec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# set(PC_AVCODEC_LIBDIR /usr/lib/x86_64-linux-gnu/)
# set(PC_AVFORMAT_LIBDIR /usr/lib/x86_64-linux-gnu/)
# set(PC_AVUTIL_LIBDIR /usr/lib/x86_64-linux-gnu/)
# # FFmpeg libraries
# if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
#     find_library(AVCODEC_LIBRARY NAMES avcodec
#             HINTS
#             ${PC_AVCODEC_LIBDIR}
#             )

#     if(NOT AVCODEC_LIBRARY )
#       message(FATAL_ERROR "avcodec library not found")
#     endif()
#     find_library(AVFORMAT_LIBRARY NAMES avformat
#             HINTS
#             ${PC_AVFORMAT_LIBDIR}
#             )
#     find_library(AVUTIL_LIBRARY NAMES avutil
#             HINTS
#             ${PC_AVUTIL_LIBDIR}
#             )

# endif()

# add_library(ffmpeg_encoder src/lib-ffmpeg_enc_dec/ffmpeg_encoder.cpp)
# target_include_directories(ffmpeg_encoder PUBLIC include)



add_subdirectory(cuvid_round_trip_test)

# add_executable(encoder_node src/encoder_node.cpp)
# target_include_directories(encoder_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(encoder_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_link_libraries(encoder_node ffmpeg_encoder)

# install(TARGETS encoder_node
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
