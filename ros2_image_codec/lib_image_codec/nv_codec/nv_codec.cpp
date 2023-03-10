#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/nv_codec/nv_codec.hpp>

// Definition of logger for NvDecoderUtil
simplelogger::Logger* logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();

namespace image_codec {

NvImageDecoder::NvImageDecoder(DecoderParams params)
    : params_(std::move(params)) {
  int iGpu = 0;
  ck(cuInit(0));
  CUdevice cuDevice = 0;
  char szDeviceName[80];
  ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
  std::cout << "GPU in use: " << szDeviceName << std::endl;
  ck(cuDeviceGet(&cuDevice, iGpu));

  ck(cuCtxCreate(&cuContext_, 0, cuDevice));
  cudaVideoCodec codec;
  if (params_.decoder_name == "h264") {
    codec = cudaVideoCodec_H264;
  } else {
    throw CodecException("Codec is currently not supported: " +
                         params_.decoder_name);
  }

  decoder_ =
      std::make_unique<NvDecoder>(cuContext_, false, codec, true, false,
                                  nullptr, nullptr, false, 0, 0, 1000, true);
}

ImageFrame NvImageDecoder::decode(const Packet& packet) {
  int num_frame_returned = decoder_->Decode(
      packet.data.data(), packet.data.size(), CUVID_PKT_ENDOFPICTURE);

  if (num_frame_returned == 0) {
    throw CodecException("No frame is returned from cuvid decoder.");
  } else if (num_frame_returned > 1) {
    throw CodecException(
        "More than one frame is returned from decoding one packet.");
  }
  int64_t timestamp;
  auto decoded_frame_data = decoder_->GetFrame(&timestamp);
  auto decoded_frame_size = decoder_->GetFrameSize();

  auto output_format = decoder_->GetOutputFormat();

  // support yuv420_nv12 only
  if (output_format != cudaVideoSurfaceFormat_NV12) {
    throw CodecException("Decoded image format is not NV12(0): " +
                         static_cast<int>(output_format));
  }

  ImageFrame output;
  output.format = "nv12";
  output.data.resize(decoded_frame_size);

  std::memcpy(output.data.data(), decoded_frame_data, decoded_frame_size);

  return output;
}
}  // namespace image_codec