#ifndef IMAGE_CODEC__NV_CODEC_HPP
#define IMAGE_CODEC__NV_CODEC_HPP

#include <lib_image_codec/i_codec.hpp>

#include "NvDecoder/NvDecoder.h"
#include "NvEncoder/NvEncoderCuda.h"
#include "NvEncoderCLIOptions.h"
namespace image_codec {

class NvImageDecoder : public IDecoder {
 public:
  NvImageDecoder(DecoderParams params){};

  NvImageDecoder(const NvImageDecoder&) = delete;
  NvImageDecoder& operator=(const NvImageDecoder&) = delete;
  NvImageDecoder(NvImageDecoder&&) = default;
  NvImageDecoder& operator=(NvImageDecoder&&) = default;

  ~NvImageDecoder();

  ImageFrame decode(const Packet& packet) override;

 private:
  CUcontext cuContext = NULL;
  std::unique_ptr<NvDecoder> decoder_;
};
}  // namespace image_codec

#endif  // IMAGE_CODEC__nv_ENCODER_HPP