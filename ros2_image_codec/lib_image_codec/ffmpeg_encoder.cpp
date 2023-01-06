extern "C" {
#include <libavutil/opt.h>
}

#include <spdlog/spdlog.h>

#include <iostream>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/ffmpeg_encoder.hpp>
namespace image_codec {

void FFmpegEncoder::test() { std::cout << "test" << std::endl; }

FFmpegEncoder::FFmpegEncoder(EncoderParams params) {
  cpu_max_align_ = av_cpu_max_align();
  encoder_ = avcodec_find_encoder_by_name(params.encoder_name.c_str());
  if (!encoder_) {
    throw EncoderException("libavcodec Encoder '" + params.encoder_name +
                           "' not found.");
  }

  encoder_context_ = avcodec_alloc_context3(encoder_);
  if (!encoder_context_) {
    throw EncoderException("libavcodec Could not allocate encoder context.");
  }

  encoder_context_->time_base = AVRational{1, 1};
  encoder_context_->framerate = AVRational{1, 1};

  encoder_context_->width = params.width;
  encoder_context_->height = params.height;
  encoder_context_->gop_size = params.gop_size;
  encoder_context_->max_b_frames = 0;
  encoder_context_->pix_fmt = AV_PIX_FMT_YUV420P;

  // Settings to enable one-in-one-out
  if (params.encoder_name == "libx264") {
    // enable one-in-one-out
    av_opt_set(encoder_context_->priv_data, "tune", "zerolatency", 0);
    av_opt_set(encoder_context_->priv_data, "crf",
               std::to_string(params.crf).c_str(), 0);

  } else if (params.encoder_name == "h264_nvenc") {
    // enable one-in-one-out
    av_opt_set(encoder_context_->priv_data, "zerolatency", "1", 0);

    if (params.crf == 0) {
      av_opt_set(encoder_context_->priv_data, "tune", "lossless", 0);
    } else {
      av_opt_set(encoder_context_->priv_data, "cq",
                 std::to_string(params.crf).c_str(), 0);
    }
  }

  output_packet_ = av_packet_alloc();
  if (!output_packet_) {
    throw EncoderException(
        "libavcodec Could not allocate encoder output packet.");
  }

  CHECK_LIBAV_ERROR(avcodec_open2(encoder_context_, encoder_, nullptr))

  init_input_frame();
}

void FFmpegEncoder::init_input_frame() {
  int avcodec_return;
  input_frame_ = av_frame_alloc();
  if (!input_frame_) {
    throw EncoderException("libavcodec Could not allocate input frame.");
  }
  input_frame_->format = encoder_context_->pix_fmt;
  input_frame_->width = encoder_context_->width;
  input_frame_->height = encoder_context_->height;

  CHECK_LIBAV_ERROR(
      av_image_check_size(input_frame_->width, input_frame_->height, 0, NULL))

  CHECK_LIBAV_ERROR(av_image_fill_linesizes(
      input_frame_->linesize, static_cast<AVPixelFormat>(input_frame_->format),
      FFALIGN(input_frame_->width, cpu_max_align_)))
}

Packet FFmpegEncoder::encode(uint8_t* input_data,
                             std::chrono::nanoseconds frame_ts) {
  return Packet{};
}

FFmpegEncoder::~FFmpegEncoder() {
  if (encoder_context_) avcodec_free_context(&encoder_context_);
  if (input_frame_) av_frame_free(&input_frame_);
  if (output_packet_) av_packet_free(&output_packet_);
}
}  // namespace image_codec