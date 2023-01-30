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
add_subdirectory(lib_image_codec)

add_executable(encoder_node codec_nodes/encoder_node.cpp)
target_include_directories(encoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(encoder_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(encoder_node image_codec)

install(TARGETS encoder_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
