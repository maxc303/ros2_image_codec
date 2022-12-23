#include <cstdio>
#include <image_enc_dec/ffmpeg_encoder.hpp>
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  image_enc_dec::Encoder encoder;
  encoder.test();
  printf("hello world image_enc_dec package\n");
  return 0;
}
