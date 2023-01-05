#include <cstdio>
#include <lib_image_codec/ffmpeg_encoder.hpp>
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  image_codec::EncoderParams encoder_params;
  image_codec::FFmpegEncoder encoder(encoder_params);
  encoder.test();
  printf("hello world image_enc_dec package\n");
  return 0;
}
