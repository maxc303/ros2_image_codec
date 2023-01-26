#ifndef IMAGE_CODEC__NV_CODEC_HPP
#define IMAGE_CODEC__NV_CODEC_HPP

#include "NvDecoder/NvDecoder.h"
#include "NvEncoder/NvEncoderCuda.h"
#include "NvEncoderCLIOptions.h"

namespace image_codec {

class NvDecoder : public IDecoder {
 public:
  NvDecoder(DecoderParams params){};

  NvDecoder(const NvDecoder&) = delete;
  NvDecoder& operator=(const NvDecoder&) = delete;
  NvDecoder(NvDecoder&&) = default;
  NvDecoder& operator=(NvDecoder&&) = default;

  ~NvDecoder();

  ImageFrame decode(const Packet& packet) override;

 private:
};
}  // namespace image_codec

#endif  // IMAGE_CODEC__nv_ENCODER_HPP