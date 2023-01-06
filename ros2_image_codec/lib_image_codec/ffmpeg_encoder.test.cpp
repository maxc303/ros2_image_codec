
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/ffmpeg_encoder.hpp>

using image_codec::EncoderParams, image_codec::FFmpegEncoder,
    image_codec::EncoderException;

TEST_CASE("FFmpegEncoder", "[unit]") {
  EncoderParams params;
  params.height = 640;
  params.width = 400;
  params.gop_size = 5;

  SECTION("Construct an Encoder") {
    SECTION("Unknown decoder") {
      params.encoder_name = "not a decoder name";
      CHECK_THROWS_AS(FFmpegEncoder(params), EncoderException);
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
      CHECK_THROWS_AS(encoder.encode(nullptr, 0), EncoderException);
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