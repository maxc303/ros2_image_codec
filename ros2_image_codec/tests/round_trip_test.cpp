
// Need to warp ffmpeg headers into extern "C"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#include <boost/program_options.hpp>
#include <filesystem>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/quality.hpp>
#include <string>

#ifdef av_err2str
#undef av_err2str
av_always_inline std::string av_err2string(int errnum) {
  char str[AV_ERROR_MAX_STRING_SIZE];
  return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#define av_err2str(err) av_err2string(err).c_str()
#endif  // av_err2str

/**
 * @brief Encode an image frame to @c AVPacket
 *
 * @param[in] enc_ctx encoder context
 * @param[in] frame
 * @param[out] pkt
 */
static void encode(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt) {
  int ret;

  /* send the frame to the encoder */
  if (frame) {
    spdlog::info("Send frame {}.", frame->pts);
  }
  ret = avcodec_send_frame(enc_ctx, frame);
  if (ret < 0) {
    spdlog::info("Error sending a frame for encoding.");
    return;
  }

  while (ret >= 0) {
    ret = avcodec_receive_packet(enc_ctx, pkt);

    if (ret == AVERROR(EAGAIN)) {
      spdlog::error(
          "Error receiving packet from encoder. Output is not available in the "
          "current state - user must try to send input.");
      av_packet_unref(pkt);
      return;
    } else if (ret == AVERROR_EOF) {
      spdlog::warn("Receive packet EOF.");
      av_packet_unref(pkt);
      return;
    } else if (ret < 0) {
      spdlog::error("Error during encoding: {}", av_err2str(ret));
      av_packet_unref(pkt);
      return;
    }

    spdlog::info("Receive packet {}, size = {} ", pkt->pts, pkt->size);
    return;
  }
}

/**
 * @brief Decode an encoded packet back to image frame
 *
 * @param[in] dec_ctx
 * @param[out] frame
 * @param[in] pkt
 */
static void decode(AVCodecContext *dec_ctx, AVFrame *frame, AVPacket *pkt) {
  int ret;

  if (pkt) {
    spdlog::info("Send pkt.");
  }
  ret = avcodec_send_packet(dec_ctx, pkt);
  if (ret < 0) {
    spdlog::error("Error sending a packet for decoding: {}.", av_err2str(ret));
    return;
  }

  while (ret >= 0 || ret == AVERROR(EAGAIN)) {
    ret = avcodec_receive_frame(dec_ctx, frame);

    if (ret == AVERROR(EAGAIN)) {
      spdlog::error(
          "Error receiving frame from decoder. Output is not available in the "
          "current state - user must try to send input.");
      return;
    } else if (ret == AVERROR_EOF) {
      spdlog::warn("Receive frame EOF.");
      return;
    } else if (ret < 0) {
      spdlog::error("Error during decoding: {}", av_err2str(ret));
      return;
    }

    spdlog::info("Receive decoded frame.", frame->pts);
    return;
  }
}

namespace po = boost::program_options;
int main(int argc, char **argv) {
  spdlog::cfg::load_env_levels();

  // Parse test infomration
  po::options_description desc(
      "FFmpeg API encoding and decoding round trip test. The program will try "
      "to encode the images from input directory and decode the packets back "
      "to images. Quality Loss of each image will be calculated. If an output "
      "directory is provided, the decoded image will be save to that "
      "directory.");
  desc.add_options()("help", "produce help message")  //
      ("input_dir", po::value<std::string>(),
       "input images dir. Only .png and .jpg images will be used.")  //
      ("output_dir", po::value<std::string>(), "output images dir")  //
      ("encoder", po::value<std::string>()->default_value("libx264"),
       "FFmpeg encoder name")  //
      ("decoder", po::value<std::string>()->default_value("h264"),
       "FFmpeg decoder name")  //
      ("num_images", po::value<int>()->default_value(10),
       "number of images to be used in this test")  //
      ("crf", po::value<int>()->default_value(23),
       "encoder bit rate")  //
      ("gop_size", po::value<int>()->default_value(10),
       "Encoder Group of Pictures size.  Emit one intra frame in each group.");

  po::variables_map boost_args;
  po::store(po::parse_command_line(argc, argv, desc), boost_args);
  po::notify(boost_args);

  if (boost_args.count("help") != 0 || boost_args.count("input_dir") == 0) {
    std::cout << desc << std::endl;
    return 1;
  }

  std::filesystem::path output_dir;
  bool save_output = false;
  if (boost_args.count("output_dir") == 0) {
    std::cout << "output_dir is not provided. No decoded image will be saved."
              << std::endl;
  } else {
    output_dir = boost_args["output_dir"].as<std::string>();
    save_output = true;
  }

  // Parse input
  std::filesystem::path input_dir = boost_args["input_dir"].as<std::string>();
  int max_num_images = boost_args["num_images"].as<int>();
  std::string encoder_name = boost_args["encoder"].as<std::string>();
  std::string decoder_name = boost_args["decoder"].as<std::string>();
  int crf = boost_args["crf"].as<int>();
  int gop_size = boost_args["gop_size"].as<int>();
  spdlog::info("Using encoder:{} and decoder:{}. crf = {}", encoder_name,
               decoder_name, crf);

  // Get the image path for the test.
  std::vector<std::filesystem::path> image_paths;
  for (const auto &entry : std::filesystem::directory_iterator(input_dir)) {
    if (entry.path().extension() == ".jpg" ||
        entry.path().extension() == ".png") {
      image_paths.push_back(entry.path());
    }
    if (image_paths.size() >= max_num_images) {
      break;
    }
  }
  if (image_paths.size() < 1) {
    spdlog::error("No image is found in the provided path: {}",
                  input_dir.string());
    return 1;
  }

  // Sort the path alphabetically
  std::sort(image_paths.begin(), image_paths.end());
  spdlog::info("Test image path (num = {}): ", image_paths.size());
  for (int idx = 0; idx < image_paths.size(); idx++) {
    spdlog::info("{}", image_paths[idx].string());
  }

  //========== ========== ========== ========== ==========
  // Encoding
  //========== ========== ========== ========== ==========
  const AVCodec *encoder;
  AVCodecContext *encoder_context = NULL;

  AVFrame *input_frame;
  AVPacket *pkt;

  encoder = avcodec_find_encoder_by_name(encoder_name.c_str());
  if (!encoder) {
    spdlog::error("Encoder {} not found.", encoder_name);
    return 1;
  }

  encoder_context = avcodec_alloc_context3(encoder);
  if (!encoder_context) {
    spdlog::error("Could not allocate video codec context.");
    return 1;
  }

  pkt = av_packet_alloc();
  if (!pkt) {
    spdlog::error("Could not allocate encoder packet.");
    return 1;
  }

  // Read the first image to get the dimension
  cv::Mat init_image = imread(image_paths[0], cv::IMREAD_COLOR);
  encoder_context->width = init_image.size().width;
  encoder_context->height = init_image.size().height;

  // Give encoder a random time_base and framerate.
  encoder_context->time_base = (AVRational){1, 10};
  encoder_context->framerate = (AVRational){10, 1};

  encoder_context->gop_size = gop_size;

  // Disable b frames to test real time encoding.
  encoder_context->max_b_frames = 0;

  // Use the format that will be used by the decoder.
  encoder_context->pix_fmt = AV_PIX_FMT_YUV420P;

  // Extra settings to make sure that frames can be retrievd without delay.
  // Use ffmpeg -h encoder={encoder-name} to list the options
  if (encoder_name == "h264_nvenc" || encoder_name == "hevc_nvenc") {
    av_opt_set(encoder_context->priv_data, "zerolatency", "1", 0);
    av_opt_set(encoder_context->priv_data, "delay", "0", 0);
    if (crf == 0) {
      av_opt_set(encoder_context->priv_data, "tune", "lossless", 0);
    } else {
      av_opt_set(encoder_context->priv_data, "cq", std::to_string(crf).c_str(),
                 0);
    }

  } else if (encoder_name == "libx264" || encoder_name == "libx265") {
    av_opt_set(encoder_context->priv_data, "tune", "zerolatency", 0);
    if (crf == 0) {
      av_opt_set(encoder_context->priv_data, "crf", "0", 0);
    } else {
      av_opt_set(encoder_context->priv_data, "crf", std::to_string(crf).c_str(),
                 0);
    }
  }

  int ret;

  // Open encoder context
  ret = avcodec_open2(encoder_context, encoder, NULL);
  if (ret < 0) {
    spdlog::error("Could not open encoder: {}", av_err2str(ret));
    return 1;
  }

  // Allocate input frame
  input_frame = av_frame_alloc();
  if (!input_frame) {
    spdlog::error("Could not allocate video frame.");
    return 1;
  }
  input_frame->format = encoder_context->pix_fmt;
  input_frame->width = encoder_context->width;
  input_frame->height = encoder_context->height;
  // Get input frame buffer
  ret = av_frame_get_buffer(input_frame, 0);
  if (ret < 0) {
    spdlog::error("Could not allocate video frame data.");
    return 1;
  }

  //
  // Main Image Encoding loop
  //
  int frame_idx = 0;
  input_frame->pict_type = AV_PICTURE_TYPE_P;
  std::vector<cv::Mat> original_bgr_images;
  std::vector<std::vector<uint8_t>> encoded_packet_data;

  for (const auto &path : image_paths) {
    cv::Mat img = imread(path, cv::IMREAD_COLOR);
    if (img.size().width != encoder_context->width ||
        img.size().height != encoder_context->height) {
      spdlog::error("Incorrect encoder image size: path = {}", path.string());
    }
    if (img.empty()) {
      spdlog::error("Could not read the image: path = {}", path.string());
      return 1;
    }

    ret = av_frame_make_writable(input_frame);

    size_t sizes[4];
    if (ret < 0) {
      spdlog::error("Could not make the frame writable: err =",
                    av_err2str(ret));
      return 1;
    }
    original_bgr_images.push_back(img);

    // Note: YUV420p planar is YUV_I420 in OpenCV
    cv::Mat img_yuv420p;
    auto t_0 = std::chrono::high_resolution_clock::now();

    cv::cvtColor(img, img_yuv420p, cv::COLOR_BGR2YUV_I420);
    auto t_1 = std::chrono::high_resolution_clock::now();

    // Copy frame data from yuv channel
    int y_channel_size = img.size().width * img.size().height;
    int uv_channel_size = img.size().width * img.size().height / 4;
    auto t_2 = std::chrono::high_resolution_clock::now();

    // input_frame->data[0] = img_yuv420p.data;
    // input_frame->data[1] = img_yuv420p.data + y_channel_size;
    // input_frame->data[2] = img_yuv420p.data + y_channel_size +
    // uv_channel_size;

    // Manually Copy input data to frame buffer
    // std::memcpy(input_frame->data[0], img_yuv420p.data, y_channel_size);
    // std::memcpy(input_frame->data[1], img_yuv420p.data + y_channel_size,
    //             uv_channel_size);
    // std::memcpy(input_frame->data[2],
    //             img_yuv420p.data + y_channel_size + uv_channel_size,
    //             uv_channel_size);

    // Fill The AVFrame with image data using av_image_fill_arrays .
    // Set the alignment to 32.
    av_image_fill_arrays(input_frame->data, input_frame->linesize,
                         img_yuv420p.data,
                         static_cast<AVPixelFormat>(input_frame->format),
                         img.size().width, img.size().height, 32);

    auto t_3 = std::chrono::high_resolution_clock::now();

    // frame timestamp = pts * timebase
    input_frame->pts = frame_idx;
    frame_idx++;

    // encode
    encode(encoder_context, input_frame, pkt);
    if (!pkt->data) {
      spdlog::info("Failed to encode frame in one call.");
      break;
    }
    auto t_4 = std::chrono::high_resolution_clock::now();

    encoded_packet_data.emplace_back(std::vector<uint8_t>(pkt->size));
    std::memcpy(encoded_packet_data.back().data(), pkt->data, pkt->size);
    auto t_5 = std::chrono::high_resolution_clock::now();

    auto t_convert_color =
        std::chrono::duration_cast<std::chrono::microseconds>(t_1 - t_0)
            .count();
    auto t_copy_to_frame =
        std::chrono::duration_cast<std::chrono::microseconds>(t_3 - t_2)
            .count();
    auto t_encode =
        std::chrono::duration_cast<std::chrono::microseconds>(t_4 - t_3)
            .count();
    auto t_copy_to_packet =
        std::chrono::duration_cast<std::chrono::microseconds>(t_5 - t_4)
            .count();

    spdlog::debug(
        "t convert color = {} us, t copy to frame = {} us, t encode = {} us, t "
        "copy to packet = {} us",
        t_convert_color, t_copy_to_frame, t_encode, t_copy_to_packet);
    // Free the packet
    av_packet_unref(pkt);
  }
  // Drain the Pipeline
  encode(encoder_context, NULL, pkt);

  int total_packet_size = 0;
  for (const auto &pkt_data : encoded_packet_data) {
    total_packet_size += pkt_data.size();
  }

  int bgr_data_size = 3 * encoder_context->width * encoder_context->height;
  int total_bgr_data_size = encoded_packet_data.size() * bgr_data_size;
  double compression_ratio = static_cast<double>(total_packet_size) /
                             static_cast<double>(total_bgr_data_size);
  spdlog::info(
      "Number of Encoded packets = {} , Total packet size = {} bytes, Total "
      "Raw BGR(8bit) size = {}, Compression Ratio = {}. ",
      encoded_packet_data.size(), total_packet_size, total_bgr_data_size,
      compression_ratio);

  //
  // Free Encoding resources
  //
  avcodec_free_context(&encoder_context);
  av_frame_free(&input_frame);
  av_packet_free(&pkt);

  //========== ========== ========== ========== ==========
  // Decoding
  //========== ========== ========== ========== ==========

  const AVCodec *decoder;
  AVCodecParserContext *parser;
  AVCodecContext *decoder_context = NULL;
  AVFrame *decoded_frame;
  AVPacket *decode_pkt;

  decode_pkt = av_packet_alloc();
  if (!decode_pkt) {
    spdlog::error("Could not allocate packet for decoding");
    return 1;
  }

  decoder = avcodec_find_decoder_by_name(decoder_name.c_str());
  if (!decoder) {
    spdlog::error("Could not find decoder: {}", decoder_name);
    return 1;
  }
  parser = av_parser_init(decoder->id);
  if (!parser) {
    spdlog::error("Could not  initialize parser: {}");
    return 1;
  }

  // Set the PARSER_FLAG_COMPLETE_FRAMES to get the parsed data before the next
  // frame arrives. The packet is known to be completed.
  parser->flags |= PARSER_FLAG_COMPLETE_FRAMES;
  decoder_context = avcodec_alloc_context3(decoder);
  if (!decoder_context) {
    spdlog::error("Could not allocate decoder context.");
    return 1;
  }

  // Add Low Delay Flag for low latency application.
  // https://github.com/FFmpeg/FFmpeg/blob/eeb280f3518a8d7fc2e45f06ac7748f42d8a0000/libavcodec/cuviddec.c#L988
  decoder_context->flags |= AV_CODEC_FLAG_LOW_DELAY;

  ret = avcodec_open2(decoder_context, decoder, NULL);
  if (ret < 0) {
    spdlog::error("Could not open decoder: {}", av_err2str(ret));
    return 1;
  }

  decoded_frame = av_frame_alloc();
  if (!decoded_frame) {
    spdlog::error("Could not allocate video frame for decoding.");
    return 1;
  }

  //
  // Main decoding loop
  //
  std::vector<cv::Mat> decoded_frames;
  int pkt_idx = 0;
  double mse_r = 0.0;
  double mse_g = 0.0;
  double mse_b = 0.0;
  double psnr_r = 0.0;
  double psnr_g = 0.0;
  double psnr_b = 0.0;
  for (auto &pkt_data : encoded_packet_data) {
    spdlog::info("Parse and Decode frame idx = {}", pkt_idx);

    auto t_0 = std::chrono::high_resolution_clock::now();

    ret = av_parser_parse2(parser, decoder_context, &decode_pkt->data,
                           &decode_pkt->size, pkt_data.data(), pkt_data.size(),
                           AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    auto t_1 = std::chrono::high_resolution_clock::now();

    if (ret < 0) {
      spdlog::error("Error while parsing packet idx = {}", pkt_idx);
      return 1;
    }

    if (!decode_pkt->size) {
      spdlog::error("Parsed unfinished packet idx = {}", pkt_idx);
      break;
    } else {
      auto t_2 = std::chrono::high_resolution_clock::now();

      decode(decoder_context, decoded_frame, decode_pkt);

      if (decoded_frame->format == -1) {
        spdlog::warn("Skip frame in decoding {}: ", pkt_idx);

        continue;
      }
      spdlog::info("Decoded frame {}, size = {}, type = {}, keyframe = {}",
                   pkt_idx, pkt_data.size(), decoded_frame->pict_type,
                   decoded_frame->key_frame ? "true" : "false");
      auto t_3 = std::chrono::high_resolution_clock::now();

      cv::Mat image(decoded_frame->height * 3 / 2, decoded_frame->width,
                    CV_8UC1);
      int y_channel_size = decoded_frame->width * decoded_frame->height;
      int uv_channel_size = decoded_frame->width * decoded_frame->height / 4;
      int image_data_size = y_channel_size + 2 * uv_channel_size;

      // // Manually Copy Frame data to cv::Mat
      // // Y channel
      // for (int row = 0; row < decoded_frame->height; row++) {
      //   std::memcpy(image.data + row * decoded_frame->width,
      //               decoded_frame->data[0] + row *
      //               decoded_frame->linesize[0], decoded_frame->width);
      // }

      // // UV channels
      // for (int row = 0; row < decoded_frame->height / 2; row++) {
      //   std::memcpy(
      //       image.data + y_channel_size + row * decoded_frame->width / 2,
      //       decoded_frame->data[1] + row * decoded_frame->linesize[1],
      //       decoded_frame->width / 2);
      //   std::memcpy(image.data + y_channel_size + uv_channel_size +
      //                   row * decoded_frame->width / 2,
      //               decoded_frame->data[2] + row *
      //               decoded_frame->linesize[2], decoded_frame->width / 2);
      // }

      // Get The AVFrame with image data using av_image_copy_to_buffer.
      // Use the same alignment as the encoder (32).
      av_image_copy_to_buffer(image.data, image_data_size, decoded_frame->data,
                              decoded_frame->linesize,
                              static_cast<AVPixelFormat>(decoded_frame->format),
                              decoded_frame->width, decoded_frame->height, 32);

      auto t_4 = std::chrono::high_resolution_clock::now();

      cv::Mat image_bgr(decoded_frame->height, decoded_frame->width, CV_8UC3);

      if (static_cast<AVPixelFormat>(decoded_frame->format) ==
          AVPixelFormat::AV_PIX_FMT_YUV420P) {
        cv::cvtColor(image, image_bgr, cv::COLOR_YUV2BGR_I420, 3);
      } else if (static_cast<AVPixelFormat>(decoded_frame->format ==
                                            AVPixelFormat::AV_PIX_FMT_NV12)) {
        cv::cvtColor(image, image_bgr, cv::COLOR_YUV2BGR_NV12, 3);
      } else {
        spdlog::error(
            "Decoded image with AVPixelFormat=[] to OpenCV conversion is not "
            "implemented in this script.",
            decoded_frame->format);
      }

      auto t_5 = std::chrono::high_resolution_clock::now();

      auto t_parse_packet =
          std::chrono::duration_cast<std::chrono::microseconds>(t_1 - t_0)
              .count();
      auto t_decode =
          std::chrono::duration_cast<std::chrono::microseconds>(t_3 - t_2)
              .count();
      auto t_get_frame =
          std::chrono::duration_cast<std::chrono::microseconds>(t_4 - t_3)
              .count();
      auto t_convert_color =
          std::chrono::duration_cast<std::chrono::microseconds>(t_5 - t_4)
              .count();

      spdlog::debug(
          "t parse packet = {} us, t decode = {} us, t get frame = {} us, t "
          "convert color = {} us.",
          t_parse_packet, t_decode, t_get_frame, t_convert_color);

      // Calculate quality metrics
      auto mse = cv::quality::QualityMSE::compute(original_bgr_images[pkt_idx],
                                                  image_bgr, cv::noArray());
      auto psnr = cv::quality::QualityPSNR::compute(
          original_bgr_images[pkt_idx], image_bgr, cv::noArray());

      mse_b += mse[0];
      mse_g += mse[1];
      mse_r += mse[2];

      psnr_b += psnr[0];
      psnr_g += psnr[1];
      psnr_r += psnr[2];

      if (save_output) {
        auto output_path = output_dir / image_paths[pkt_idx].filename();
        cv::imwrite(output_path, image_bgr);
        spdlog::debug("Saved Image to :{}", output_path.string());
      }
    }
    pkt_idx++;
  }

  mse_b /= encoded_packet_data.size();
  mse_g /= encoded_packet_data.size();
  mse_r /= encoded_packet_data.size();

  psnr_b /= encoded_packet_data.size();
  psnr_g /= encoded_packet_data.size();
  psnr_r /= encoded_packet_data.size();

  spdlog::info("Average Image quality scores:");
  spdlog::info("BGR MSE (Lower is better) = [{}, {}, {}]", mse_b, mse_g, mse_r);
  spdlog::info("BGR PSNR (Should be ~30-50dB, higher is better) = [{}, {}, {}]",
               psnr_b, psnr_g, psnr_r);

  av_parser_close(parser);
  avcodec_free_context(&decoder_context);
  av_frame_free(&decoded_frame);
  av_packet_free(&decode_pkt);

  return 0;
}