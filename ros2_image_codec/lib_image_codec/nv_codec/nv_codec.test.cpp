#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/nv_codec/nv_codec.hpp>

using image_codec::NvImageDecoder, image_codec::DecoderParams;

TEST_CASE("nv_image_codec", "[unit]") {
  DecoderParams decoder_params;
  decoder_params.decoder_name = "h264";

  NvImageDecoder decoder(decoder_params);
}