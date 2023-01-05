#ifndef IMAGE_CODEC__FFMPEG_ENCODER_HPP
#define IMAGE_CODEC__FFMPEG_ENCODER_HPP

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
}
#include <chrono>
#include <lib_image_codec/i_encoder.hpp>
#include <string>
#include <vector>
namespace image_codec {

class FFmpegEncoder : public IEncoder {
 public:
  void test();
  FFmpegEncoder(EncoderParams params);
  Packet encode(uint8_t* input_data, std::chrono::nanoseconds frame_ts) {
    return Packet{};
  };

 private:
  const AVCodec* encoder_;
  AVFrame* input_frame_;
  AVPacket* output_packet_;
};

}  // namespace image_codec

#endif  // IMAGE_CODEC__FFMPEG_ENCODER_HPP