#ifndef IMAGE_CODEC__FFMPEG_CODEC_HPP
#define IMAGE_CODEC__FFMPEG_CODEC_HPP

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
}
#include <chrono>
#include <lib_image_codec/i_codec.hpp>
#include <string>
#include <vector>
namespace image_codec {

class FFmpegEncoder : public IEncoder {
 public:
  FFmpegEncoder(EncoderParams params);

  FFmpegEncoder(const FFmpegEncoder&) = delete;
  FFmpegEncoder& operator=(const FFmpegEncoder&) = delete;
  FFmpegEncoder(FFmpegEncoder&&) = default;
  FFmpegEncoder& operator=(FFmpegEncoder&&) = default;

  ~FFmpegEncoder();
  Packet encode(uint8_t* input_data, size_t data_size) override;

 private:
  void init_input_frame();
  const AVCodec* encoder_;
  AVCodecContext* encoder_context_;
  AVFrame* input_frame_;
  AVPacket* output_packet_;
  int pts_ = 0;

  int cpu_max_align_;
  EncoderParams params_;
};

class FFmpegDecoder : public IDecoder {
 public:
  FFmpegDecoder(DecoderParams params);

  FFmpegDecoder(const FFmpegDecoder&) = delete;
  FFmpegDecoder& operator=(const FFmpegDecoder&) = delete;
  FFmpegDecoder(FFmpegDecoder&&) = default;
  FFmpegDecoder& operator=(FFmpegDecoder&&) = default;

  ~FFmpegDecoder();

  ImageFrame decode(const Packet& packet) override;

 private:
  DecoderParams params_;
  const AVCodec* decoder_;
  AVCodecParserContext* parser_;
  AVCodecContext* decoder_context_;
  AVPacket* input_packet_;
  AVFrame* output_frame_;
};
}  // namespace image_codec

#endif  // IMAGE_CODEC__FFMPEG_ENCODER_HPP