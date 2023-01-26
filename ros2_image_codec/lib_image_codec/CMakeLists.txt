add_subdirectory(ffmpeg_codec)
add_subdirectory(nv_codec)

add_library(image_codec INTERFACE)

target_link_libraries(image_codec INTERFACE ffmpeg_codec nv_codec)