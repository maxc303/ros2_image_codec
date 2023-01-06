#ifndef IMAGE_CODEC__I_ENCODER_HPP
#define IMAGE_CODEC__I_ENCODER_HPP

#include <chrono>
#include <string>
#include <vector>

namespace image_codec {

struct EncoderParams {
  std::string encoder_name;
  int gop_size;
  int width;
  int height;
  int crf = 23;
};

struct Packet {
  std::vector<uint8_t> data;

  // Timestamped to check previous frame.
  std::chrono::nanoseconds previous_frame_ts;
};

class IEncoder {
 public:
  virtual ~IEncoder() = default;

  virtual Packet encode(uint8_t* input_data,
                        std::chrono::nanoseconds frame_ts) = 0;
};

}  // namespace image_codec
#endif  // IMAGE_CODEC__I_ENCODER_HPP