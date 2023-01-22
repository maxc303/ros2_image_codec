
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/ffmpeg_codec.hpp>

using image_codec::EncoderParams, image_codec::FFmpegEncoder,
    image_codec::CodecException, image_codec::DecoderParams,
    image_codec::FFmpegDecoder;

TEST_CASE("FFmpegEncoder", "[unit]") {
  EncoderParams params;
  params.height = 640;
  params.width = 480;
  params.gop_size = 5;

  SECTION("Construct an Encoder") {
    SECTION("Unknown decoder") {
      params.encoder_name = "not a decoder name";
      CHECK_THROWS_AS(FFmpegEncoder(params), CodecException);
    }

    SECTION("Construct libx264 Encoder") {
      params.encoder_name = "libx264";
      REQUIRE_NOTHROW(FFmpegEncoder(params));
    }
  }

  SECTION("Encode") {
    params.encoder_name = "libx264";
    FFmpegEncoder encoder(params);

    SECTION("null input") {
      CHECK_THROWS_AS(encoder.encode(nullptr, 0), CodecException);
    }
    SECTION("Good input") {
      // Initialize a YUV data vector that all the value is 128.
      std::vector<uint8_t> input_data(params.height * params.width * 3 / 2,
                                      128);

      auto packet = encoder.encode(input_data.data(), input_data.size());
      CHECK(packet.data.size() > 0);
      CHECK(packet.is_key);
    }
  }
}

/**
 * Utility function to test an ffmpeg encoder + decoder combination
 */
void test_enc_dec(const std::string &encoder_name,
                  const std::string &decoder_name) {
  EncoderParams enc_params;
  enc_params.height = 640;
  enc_params.width = 480;
  enc_params.gop_size = 5;
  enc_params.encoder_name = encoder_name;
  std::vector<uint8_t> input_data(enc_params.height * enc_params.width * 3 / 2,
                                  128);
  FFmpegEncoder encoder(enc_params);

  DecoderParams dec_params;
  dec_params.decoder_name = decoder_name;
  FFmpegDecoder decoder(dec_params);

  for (int i = 0; i < 5; i++) {
    auto packet = encoder.encode(input_data.data(), input_data.size());

    CHECK(packet.data.size() > 0);
    CHECK(packet.data.size() < input_data.size());

    image_codec::ImageFrame decoded_image = decoder.decode(packet);
    CHECK(decoded_image.data.size() == input_data.size());
    CHECK(decoded_image.data == input_data);
  }
}

TEST_CASE("FFmpeg Encode and Decode", "[unit]") {
  SECTION("mjpeg + mjpeg") { test_enc_dec("mjpeg", "mjpeg"); }
  SECTION("mjpeg + mjpeg_cuvid") { test_enc_dec("mjpeg", "mjpeg_cuvid"); }
  SECTION("libx264 + h264") { test_enc_dec("libx264", "h264"); }
  SECTION("libx264 + h264_cuvid") { test_enc_dec("libx264", "h264_cuvid"); }
  SECTION("libx265 + h265") { test_enc_dec("libx265", "hevc"); }
}