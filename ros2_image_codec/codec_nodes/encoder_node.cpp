#include <cstdio>
#include <iostream>
#include <lib_image_codec/ffmpeg_codec/ffmpeg_codec.hpp>
#include <lib_image_codec/nv_codec/nv_codec.hpp>
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  // image_codec::EncoderParams encoder_params;
  // encoder_params.encoder_name = "libx264";
  // encoder_params.height = 640;
  // encoder_params.width = 400;

  // image_codec::FFmpegEncoder encoder(encoder_params);

  // image_codec::DecoderParams decoder_params;
  // decoder_params.decoder_name = "h264";

  // image_codec::FFmpegDecoder decoder(decoder_params);
  printf("hello world image_enc_dec package\n");

  image_codec::EncoderParams enc_params;
  enc_params.height = 900;
  enc_params.width = 1600;
  enc_params.gop_size = 5;
  enc_params.encoder_name = "libx264";
  std::vector<uint8_t> input_data(enc_params.height * enc_params.width * 3 / 2,
                                  128);
  image_codec::FFmpegEncoder encoder(enc_params);
  auto packet = encoder.encode(input_data.data(), input_data.size());

  image_codec::DecoderParams dec_params;
  dec_params.decoder_name = "h264";
  image_codec::FFmpegDecoder decoder(dec_params);
  image_codec::ImageFrame decoded_image = decoder.decode(packet);
  std::cout << decoded_image.data.size() << std::endl;
  return 0;
}
