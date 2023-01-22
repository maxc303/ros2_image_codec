#ifndef IMAGE_CODEC__I_ENCODER_HPP
#define IMAGE_CODEC__I_ENCODER_HPP

#include <chrono>
#include <string>
#include <vector>

namespace image_codec {

struct EncoderParams {
  std::string encoder_name;
  bool i_frame_only = true;
  int gop_size;
  int width;
  int height;
  int crf = 23;    // FFmpeg libx264/libx265 quality factor [0-51]. Lower is
                   // better
  int qscale = 5;  // FFmpeg jpeg quality factor [1-31]. Lower is better
};

struct DecoderParams {
  std::string decoder_name;
};

struct Packet {
  std::vector<uint8_t> data;

  bool is_key = false;
  // Timestamped to check previous frame.
  std::chrono::nanoseconds previous_frame_ts;
};

struct ImageFrame {
  std::vector<uint8_t> data;
  std::string format;
};

class IEncoder {
 public:
  virtual ~IEncoder() = default;

  virtual Packet encode(uint8_t* input_data, size_t data_size) = 0;
};

class IDecoder {
 public:
  virtual ~IDecoder() = default;
  virtual ImageFrame decode(const Packet& packet) = 0;
};

}  // namespace image_codec
#endif  // IMAGE_CODEC__I_ENCODER_HPP