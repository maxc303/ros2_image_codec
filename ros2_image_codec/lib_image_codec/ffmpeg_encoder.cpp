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

  CHECK_LIBAV_ERROR(av_frame_get_buffer(input_frame_, 0))

  // // Fill linesizes if the frame buffer is initialized on the input data.
  // CHECK_LIBAV_ERROR(
  //     av_image_check_size(input_frame_->width, input_frame_->height, 0,
  //     NULL))

  // CHECK_LIBAV_ERROR(av_image_fill_linesizes(
  //     input_frame_->linesize,
  //     static_cast<AVPixelFormat>(input_frame_->format),
  //     FFALIGN(input_frame_->width, cpu_max_align_)))
}

Packet FFmpegEncoder::encode(uint8_t* input_data, size_t data_size) {
  if (!input_data) {
    throw EncoderException("Input data can't' be null.");
  }
  CHECK_LIBAV_ERROR(av_image_fill_arrays(
      input_frame_->data, input_frame_->linesize, input_data,
      static_cast<AVPixelFormat>(input_frame_->format), input_frame_->width,
      input_frame_->height, cpu_max_align_))

  // // Use the input data as the buffer.
  // input_frame_->buf[0] =
  //     av_buffer_create(input_data, data_size, nullptr, nullptr, 0);
  // if (!input_frame_->buf[0]) {
  //   throw LibavException("Failed to create frame buffer from input
  //   data.");
  // }
  // int padded_height = FFALIGN(input_frame_->height, 32);
  // // Fill data from buffer
  // CHECK_LIBAV_ERROR(av_image_fill_pointers(
  //     input_frame_->data,
  //     static_cast<AVPixelFormat>(input_frame_->format), padded_height,
  //     input_frame_->buf[0]->data, input_frame_->linesize))

  // Use pts as index
  input_frame_->pts = pts_;
  pts_++;

  CHECK_LIBAV_ERROR(avcodec_send_frame(encoder_context_, input_frame_))

  int receive_packet_return =
      avcodec_receive_packet(encoder_context_, output_packet_);
  if (receive_packet_return == AVERROR(EAGAIN)) {
    throw EncoderException(
        "Could not receive a packet from one input frame. Check the encoder "
        "setting to ensure one-in-one-out.");
  } else if (receive_packet_return == AVERROR_EOF) {
    throw EncoderException(
        "Receive Packet EOF. This should not happen during encoding.");
  } else if (receive_packet_return < 0) {
    throw LibavException("Libav error during encoding: " +
                         av_err2string(receive_packet_return));
  }

  Packet encoded_packet;
  encoded_packet.data.resize(output_packet_->size);
  std::memcpy(encoded_packet.data.data(), output_packet_->data,
              output_packet_->size);

  encoded_packet.is_key = output_packet_->flags & AV_PKT_FLAG_KEY;
  return encoded_packet;
}

FFmpegEncoder::~FFmpegEncoder() {
  if (encoder_context_) avcodec_free_context(&encoder_context_);
  if (input_frame_) av_frame_free(&input_frame_);
  if (output_packet_) av_packet_free(&output_packet_);
}
}  // namespace image_codec