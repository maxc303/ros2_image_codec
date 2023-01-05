#ifndef IMAGE_ENC_DEC__FFMPEG_ENCODER_HPP
#define IMAGE_ENC_DEC__FFMPEG_ENCODER_HPP

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
}
#include <chrono>
#include <string>
#include <vector>
namespace ffmpeg_encoder {

struct EncoderParams {
  std::string name;
  int bit_rate;
  int width;
  int height;
  int gop_size;
};

struct Packet {
  std::vector<uint8_t> data;

  // Timestamped to check previous frame.
  std::chrono::nanoseconds previous_frame_ts;
};

class Encoder {
 public:
  void test();
  Encoder(EncoderParams params){};
  Packet encode(uint8_t* input_data, std::chrono::nanoseconds frame_ts){};

 private:
  const AVCodec* encoder_;
  AVFrame* input_frame_;
  AVPacket* output_packet_;
};

}  // namespace ffmpeg_encoder

#endif  // IMAGE_ENC_DEC__FFMPEG_ENCODER_HPP