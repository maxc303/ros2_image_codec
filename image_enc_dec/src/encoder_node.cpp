#include <cstdio>
#include <image_enc_dec/ffmpeg_encoder.hpp>
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  ffmpeg_encoder::EncoderParams encoder_params;
  ffmpeg_encoder::Encoder encoder(encoder_params);
  encoder.test();
  printf("hello world image_enc_dec package\n");
  return 0;
}
