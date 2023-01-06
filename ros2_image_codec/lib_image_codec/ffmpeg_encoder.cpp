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
  if (encoder_->name == "libx264") {
    // enable one-in-one-out
    av_opt_set(encoder_context_->priv_data, "tune", "zerolatency", 0);
    av_opt_set(encoder_context_->priv_data, "crf",
               std::to_string(params.crf).c_str(), 0);

  } else if (encoder_->name == "h264_nvenc") {
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

  int avcodec_return;
  avcodec_return = avcodec_open2(encoder_context_, encoder_, nullptr);
  if (avcodec_return < 0) {
    throw EncoderException("libavcodec Could not open encoder. Error type: " +
                           av_err2string(avcodec_return));
  }

  input_frame_ = av_frame_alloc();
  if (!input_frame_) {
    throw EncoderException("libavcodec Could not allocate input frame.");
  }

  input_frame_->format = encoder_context_->pix_fmt;
  input_frame_->width = encoder_context_->width;
  input_frame_->height = encoder_context_->height;
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