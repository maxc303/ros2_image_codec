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

  FFmpegEncoder(const FFmpegEncoder&) = delete;
  FFmpegEncoder& operator=(const FFmpegEncoder&) = delete;
  FFmpegEncoder(FFmpegEncoder&&) = default;
  FFmpegEncoder& operator=(FFmpegEncoder&&) = default;

  ~FFmpegEncoder();
  Packet encode(uint8_t* input_data, std::chrono::nanoseconds frame_ts);

 private:
  void init_input_frame();
  const AVCodec* encoder_;
  AVCodecContext* encoder_context_;
  AVFrame* input_frame_;
  AVPacket* output_packet_;
  int dts = 0;

  int cpu_max_align_;
};

}  // namespace image_codec

#endif  // IMAGE_CODEC__FFMPEG_ENCODER_HPP