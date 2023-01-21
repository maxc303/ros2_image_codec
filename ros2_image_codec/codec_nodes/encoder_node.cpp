#include <cstdio>
#include <lib_image_codec/ffmpeg_codec.hpp>
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  image_codec::EncoderParams encoder_params;
  encoder_params.encoder_name = "libx264";
  encoder_params.height = 640;
  encoder_params.width = 400;

  image_codec::FFmpegEncoder encoder(encoder_params);

  image_codec::DecoderParams decoder_params;
  decoder_params.decoder_name = "h264";

  image_codec::FFmpegDecoder decoder(decoder_params);
  printf("hello world image_enc_dec package\n");
  return 0;
}
