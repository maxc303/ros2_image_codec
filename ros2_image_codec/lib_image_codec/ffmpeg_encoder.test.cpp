
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/ffmpeg_encoder.hpp>

using image_codec::EncoderParams, image_codec::FFmpegEncoder,
    image_codec::EncoderException;

TEST_CASE("FFmpegEncoder", "[unit]") {
  EncoderParams params;
  params.height = 1920;
  params.width = 1080;
  params.gop_size = 5;

  SECTION("Unknown decoder") {
    params.encoder_name = "not a decoder";
    REQUIRE_THROWS_AS(FFmpegEncoder(params), EncoderException);
  }

  SECTION("Construct x264 Encoder") {
    params.encoder_name = "libx264";
    REQUIRE_NOTHROW(FFmpegEncoder(params));
  }
}