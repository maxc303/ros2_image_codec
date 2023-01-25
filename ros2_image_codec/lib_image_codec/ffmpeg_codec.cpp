extern "C" {
#include <libavutil/opt.h>
}

#include <spdlog/spdlog.h>

#include <iostream>
#include <lib_image_codec/exceptions.hpp>
#include <lib_image_codec/ffmpeg_codec.hpp>
namespace image_codec {

FFmpegEncoder::FFmpegEncoder(EncoderParams params)
    : params_(std::move(params)) {
  cpu_max_align_ = av_cpu_max_align();
  encoder_ = avcodec_find_encoder_by_name(params_.encoder_name.c_str());
  if (!encoder_) {
    throw CodecException("libavcodec Encoder '" + params_.encoder_name +
                         "' not found.");
  }

  encoder_context_ = avcodec_alloc_context3(encoder_);
  if (!encoder_context_) {
    throw CodecException("libavcodec Could not allocate encoder context.");
  }

  encoder_context_->time_base = AVRational{1, 1};
  encoder_context_->framerate = AVRational{1, 1};

  encoder_context_->width = params_.width;
  encoder_context_->height = params_.height;
  encoder_context_->gop_size = params.gop_size;
  encoder_context_->max_b_frames = 0;

  encoder_context_->pix_fmt = AV_PIX_FMT_YUV420P;

  // Settings to enable one-in-one-out
  if (params_.encoder_name == "libx264" || params_.encoder_name == "libx265") {
    // enable one-in-one-out
    av_opt_set(encoder_context_->priv_data, "tune", "zerolatency", 0);
    av_opt_set(encoder_context_->priv_data, "crf",
               std::to_string(params_.crf).c_str(), 0);

  } else if (params_.encoder_name == "h264_nvenc") {
    // enable one-in-one-out
    av_opt_set(encoder_context_->priv_data, "zerolatency", "1", 0);

    if (params_.crf == 0) {
      av_opt_set(encoder_context_->priv_data, "tune", "lossless", 0);
    } else {
      av_opt_set(encoder_context_->priv_data, "cq",
                 std::to_string(params_.crf).c_str(), 0);
    }
  } else if (params_.encoder_name == "mjpeg") {
    encoder_context_->qmin = 1;
    // mjpeg uses YUVJ420P, which has wider value range than YUV420P
    encoder_context_->pix_fmt = AV_PIX_FMT_YUVJ420P;
    encoder_context_->flags |= AV_CODEC_FLAG_QSCALE;
    encoder_context_->global_quality = params_.qscale * FF_QP2LAMBDA;
  }

  output_packet_ = av_packet_alloc();
  if (!output_packet_) {
    throw CodecException(
        "libavcodec Could not allocate encoder output packet.");
  }

  CHECK_LIBAV_ERROR(avcodec_open2(encoder_context_, encoder_, nullptr))

  init_input_frame();
}

void FFmpegEncoder::init_input_frame() {
  int avcodec_return;
  input_frame_ = av_frame_alloc();
  if (!input_frame_) {
    throw CodecException("libavcodec Could not allocate input frame.");
  }
  input_frame_->format = encoder_context_->pix_fmt;
  input_frame_->width = encoder_context_->width;
  input_frame_->height = encoder_context_->height;

  if (params_.use_input_as_buf) {
    // Fill linesizes if the frame buffer is initialized on the input data.
    CHECK_LIBAV_ERROR(
        av_image_check_size(input_frame_->width, input_frame_->height, 0, NULL))

    CHECK_LIBAV_ERROR(av_image_fill_linesizes(
        input_frame_->linesize,
        static_cast<AVPixelFormat>(input_frame_->format),
        FFALIGN(input_frame_->width, 1)))
  } else {
    CHECK_LIBAV_ERROR(av_frame_get_buffer(input_frame_, 0))
  }
}

Packet FFmpegEncoder::encode(uint8_t* input_data, size_t data_size) {
  if (!input_data) {
    throw CodecException("Input data can't' be null.");
  }

  if (params_.use_input_as_buf) {
    // Use the input data as the buffer.
    input_frame_->buf[0] =
        av_buffer_create(input_data, data_size, nullptr, nullptr, 0);
    if (!input_frame_->buf[0]) {
      throw LibavException("Failed to create frame buffer from input data.");
    }
    // // int padded_height = FFALIGN(input_frame_->height, 32);
    // Fill data from buffer
    CHECK_LIBAV_ERROR(av_image_fill_pointers(
        input_frame_->data, static_cast<AVPixelFormat>(input_frame_->format),
        input_frame_->height, input_frame_->buf[0]->data,
        input_frame_->linesize))

  } else {
    // Copy data from input to image buffer
    // Note: Using a specific alignment may improve the performance.
    CHECK_LIBAV_ERROR(av_image_fill_arrays(
        input_frame_->data, input_frame_->linesize, input_data,
        static_cast<AVPixelFormat>(input_frame_->format), input_frame_->width,
        input_frame_->height, 1))
  }

  // Use pts as index
  input_frame_->pts = pts_;
  pts_++;

  if (params_.i_frame_only) {
    input_frame_->pict_type = AV_PICTURE_TYPE_I;
  }

  CHECK_LIBAV_ERROR(avcodec_send_frame(encoder_context_, input_frame_))

  int receive_packet_return =
      avcodec_receive_packet(encoder_context_, output_packet_);
  if (receive_packet_return == AVERROR(EAGAIN)) {
    throw CodecException(
        "Could not receive a packet from one input frame. Check the encoder "
        "setting to ensure one-in-one-out.");
  } else if (receive_packet_return == AVERROR_EOF) {
    throw CodecException(
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
  // When the input data is used as buffer, the memery should not be free by the
  // frame.
  if (input_frame_ && !params_.use_input_as_buf) av_frame_free(&input_frame_);
  if (output_packet_) av_packet_free(&output_packet_);
}

//
//
// Decoder
//
//

FFmpegDecoder::FFmpegDecoder(DecoderParams params)
    : params_(std::move(params)) {
  input_packet_ = av_packet_alloc();
  if (!input_packet_) {
    throw CodecException("Failed to allocate packet for decoder input.");
  }
  decoder_ = avcodec_find_decoder_by_name(params_.decoder_name.c_str());
  if (!decoder_) {
    throw CodecException("libavcodec Decoder '" + params_.decoder_name +
                         "' not found.");
  }

  parser_ = av_parser_init(decoder_->id);
  if (!parser_) {
    throw CodecException("Failed to init packet parser.");
  }
  // The packet is known to be completed. Set the PARSER_FLAG_COMPLETE_FRAMES to
  // get the parsed data before the next frame arrives.
  parser_->flags |= PARSER_FLAG_COMPLETE_FRAMES;

  decoder_context_ = avcodec_alloc_context3(decoder_);
  if (!decoder_context_) {
    throw CodecException("Failed to allocate decoder context.");
  }

  CHECK_LIBAV_ERROR(avcodec_open2(decoder_context_, decoder_, NULL));

  output_frame_ = av_frame_alloc();
  if (!output_frame_) {
    throw CodecException("Failed to allocate buffer for decoder output.");
  }
}

FFmpegDecoder::~FFmpegDecoder() {
  if (parser_) av_parser_close(parser_);
  if (decoder_context_) avcodec_free_context(&decoder_context_);
  if (output_frame_) av_frame_free(&output_frame_);
  if (input_packet_) av_packet_free(&input_packet_);
}

ImageFrame FFmpegDecoder::decode(const Packet& packet) {
  // Parse packet data to av_packet
  CHECK_LIBAV_ERROR(av_parser_parse2(parser_, decoder_context_,
                                     &input_packet_->data, &input_packet_->size,
                                     packet.data.data(), packet.data.size(),
                                     AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0))
  if (!input_packet_->size) {
    throw CodecException("Parsed incomplete packet");
  }

  CHECK_LIBAV_ERROR(avcodec_send_packet(decoder_context_, input_packet_))

  int receive_frame_return =
      avcodec_receive_frame(decoder_context_, output_frame_);

  if (receive_frame_return == AVERROR(EAGAIN)) {
    // A work around for *_cuvid decoders to achieve one-in-one-out.
    // The way to set cuvid packet flag CUVID_PKT_ENDOFPICTURE from libav is not
    // found yet.
    // Flushing is slow.
    CHECK_LIBAV_ERROR(avcodec_send_packet(decoder_context_, nullptr))

    receive_frame_return =
        avcodec_receive_frame(decoder_context_, output_frame_);
    if (receive_frame_return == AVERROR(EAGAIN)) {
      throw CodecException(
          "Could not receive a frame from one input packet. Check the decoder "
          "setting to ensure one-in-one-out.");
    }
    avcodec_flush_buffers(decoder_context_);
  } else if (receive_frame_return == AVERROR_EOF) {
    throw CodecException(
        "Receive Frame EOF. This should not happen during encoding.");
  } else if (receive_frame_return < 0) {
    throw LibavException("Libav error during decoding: " +
                         av_err2string(receive_frame_return));
  }

  ImageFrame output;
  if (static_cast<AVPixelFormat>(output_frame_->format) ==
          AVPixelFormat::AV_PIX_FMT_YUV420P ||
      static_cast<AVPixelFormat>(output_frame_->format) ==
          AVPixelFormat::AV_PIX_FMT_YUVJ420P) {
    output.format = "yuv420p";
  } else if (static_cast<AVPixelFormat>(output_frame_->format) ==
             AVPixelFormat::AV_PIX_FMT_NV12) {
    output.format = "nv12";
  } else {
    throw CodecException("Output frame format is not supported: " +
                         std::to_string(output_frame_->format));
  }
  int buffer_size = av_image_get_buffer_size(
      static_cast<AVPixelFormat>(output_frame_->format), output_frame_->width,
      output_frame_->height, 1);
  output.data.resize(buffer_size);

  av_image_copy_to_buffer(output.data.data(), buffer_size, output_frame_->data,
                          output_frame_->linesize,
                          static_cast<AVPixelFormat>(output_frame_->format),
                          output_frame_->width, output_frame_->height, 1);
  return output;
}

}  // namespace image_codec