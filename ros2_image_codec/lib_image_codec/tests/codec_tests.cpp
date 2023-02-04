#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/ffmpeg_codec/ffmpeg_codec.hpp>
#include <lib_image_codec/nv_codec/nv_codec.hpp>

using image_codec::EncoderParams, image_codec::FFmpegEncoder,
    image_codec::CodecException, image_codec::DecoderParams,
    image_codec::FFmpegDecoder, image_codec::IEncoder, image_codec::IDecoder,
    image_codec::NvImageDecoder;
;

/**
 * Utility function to test an ffmpeg encoder + decoder combination
 */
void test_enc_dec(const std::string &encoder_name, std::string encoder_type,
                  const std::string &decoder_name, std::string decoder_type) {
  EncoderParams enc_params;
  enc_params.height = 640;
  enc_params.width = 480;
  enc_params.gop_size = 5;
  enc_params.encoder_name = encoder_name;
  std::vector<uint8_t> input_data(enc_params.height * enc_params.width * 3 / 2,
                                  128);

  std::unique_ptr<IEncoder> encoder;

  if (encoder_type == "ffmpeg") {
    encoder = std::make_unique<FFmpegEncoder>(enc_params);
  } else {
    throw std::invalid_argument("Encoder type not supported: " + encoder_type);
  }

  DecoderParams dec_params;
  dec_params.decoder_name = decoder_name;

  std::unique_ptr<IDecoder> decoder;
  if (decoder_type == "ffmpeg") {
    decoder = std::make_unique<FFmpegDecoder>(dec_params);
  } else if (decoder_type == "cuvid") {
    decoder = std::make_unique<NvImageDecoder>(dec_params);
  }

  for (int i = 0; i < 5; i++) {
    auto packet = encoder->encode(input_data.data(), input_data.size());

    CHECK(packet.data.size() > 0);
    CHECK(packet.data.size() < input_data.size());

    image_codec::ImageFrame decoded_image = decoder->decode(packet);
    CHECK(decoded_image.data.size() == input_data.size());
    CHECK(decoded_image.data == input_data);
  }
}

TEST_CASE("FFmpeg Encode and Decode", "[unit]") {
  SECTION("ffmpeg libx264 + ffmpeg h264") {
    test_enc_dec("libx264", "ffmpeg", "h264", "ffmpeg");
  }
  SECTION("ffmpeg libx264 + h264_cuvid") {
    test_enc_dec("libx264", "ffmpeg", "h264_cuvid", "ffmpeg");
  }
  SECTION("ffmpeg libx264 + cuvid h264") {
    test_enc_dec("libx264", "ffmpeg", "h264", "cuvid");
  }
}