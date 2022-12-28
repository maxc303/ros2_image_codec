
// Need to warp ffmpeg headers into extern "C"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include <boost/program_options.hpp>
#include <filesystem>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/quality.hpp>
#include <string>

#ifdef av_err2str
#undef av_err2str
#include <string>
av_always_inline std::string av_err2string(int errnum) {
  char str[AV_ERROR_MAX_STRING_SIZE];
  return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#define av_err2str(err) av_err2string(err).c_str()
#endif  // av_err2str

static void encode(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt) {
  int ret;

  /* send the frame to the encoder */
  if (frame) printf("Send frame %3" PRId64 "\n", frame->pts);

  ret = avcodec_send_frame(enc_ctx, frame);
  if (ret < 0) {
    fprintf(stderr, "Error sending a frame for encoding\n");
    exit(1);
  }
  while (ret >= 0) {
    ret = avcodec_receive_packet(enc_ctx, pkt);

    std::cout << "Return code: " << ret << std::endl;
    if (ret == AVERROR(EAGAIN)) {
      return;
    } else if (ret == AVERROR_EOF) {
      av_packet_unref(pkt);
      return;
    } else if (ret < 0) {
      fprintf(stderr, "Error during encoding\n");
      exit(1);
    }

    printf("Write packet %3" PRId64 " (size=%5d)\n", pkt->pts, pkt->size);
    return;
  }
}

static void decode(AVCodecContext *dec_ctx, AVFrame *frame, AVPacket *pkt) {
  int ret;
  ret = avcodec_send_packet(dec_ctx, pkt);
  if (ret < 0) {
    fprintf(stderr, "Error sending a packet for decoding  %s\n",
            av_err2str(ret));
    exit(1);
  }
  std::cout << "Sent a packet" << std::endl;

  while (ret >= 0 || ret == AVERROR(EAGAIN)) {
    ret = avcodec_receive_frame(dec_ctx, frame);
    std::cout << "Return code :" << ret << "=" << av_err2str(ret) << std::endl;
    if (ret == AVERROR(EAGAIN)) {
      return;
    } else if (ret == AVERROR_EOF)
      return;
    else if (ret < 0) {
      fprintf(stderr, "Error during decoding\n");
      exit(1);
    }

    printf("saving frame %3d\n", dec_ctx->frame_number);
    fflush(stdout);
    return;
  }
}

namespace po = boost::program_options;
int main(int argc, char **argv) {
  // Parse test infomration
  po::options_description desc(
      "FFmpeg API encoding and decoding round trip test");
  desc.add_options()("help", "produce help message")               //
      ("input_dir", po::value<std::string>(), "input images dir")  //
      ("output_dir", po::value<std::string>(), "output images dir");

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
  std::filesystem::path input_dir = boost_args["input_dir"].as<std::string>();
  std::vector<std::filesystem::path> image_paths;

  int max_num_images = 10;
  for (const auto &entry : std::filesystem::directory_iterator(input_dir)) {
    image_paths.push_back(entry.path());
    if (image_paths.size() >= max_num_images) {
      break;
    }
  }
  std::sort(image_paths.begin(), image_paths.end());
  for (const auto &path : image_paths) {
    std::cout << path << std::endl;
  }

  /*
   * Encoding
   *
   *
   *
   *
   */
  const AVCodec *encoder;
  AVCodecContext *encoder_context = NULL;

  AVFrame *frame;
  AVPacket *pkt;

  std::string encoder_name = "h264_nvenc";
  encoder = avcodec_find_encoder_by_name(encoder_name.c_str());
  if (!encoder) {
    fprintf(stderr, "Codec '%s' not found\n", encoder_name.c_str());
    exit(1);
  }

  encoder_context = avcodec_alloc_context3(encoder);
  if (!encoder_context) {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }

  pkt = av_packet_alloc();

  if (!pkt) return 1;

  /* put sample parameters */
  encoder_context->bit_rate = 4000000;
  /* resolution must be a multiple of two */
  encoder_context->width = 1600;
  encoder_context->height = 900;
  /* frames per second */
  encoder_context->time_base = (AVRational){1, 10};
  encoder_context->framerate = (AVRational){10, 1};
  /* emit one intra frame every ten frames
   * check frame pict_type before passing frame
   * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
   * then gop_size is ignored and the output of encoder
   * will always be I frame irrespective to gop_size
   */
  encoder_context->gop_size = 5;
  encoder_context->max_b_frames = 0;
  encoder_context->pix_fmt = AV_PIX_FMT_YUV420P;

  // Extra settings to enable frame can be retrievd without waiting for more
  // frames.
  // Use ffmpeg -h encoder={encoder-name} to list the options
  if (encoder_name == "h264_nvenc") {
    av_opt_set(encoder_context->priv_data, "zerolatency", "1", 0);
    av_opt_set(encoder_context->priv_data, "delay", "0", 0);
  } else if (encoder_name == "libx264") {
    av_opt_set(encoder_context->priv_data, "tune", "zerolatency", 0);
  }
  int ret;
  ret = avcodec_open2(encoder_context, encoder, NULL);
  if (ret < 0) {
    fprintf(stderr, "Could not open codec: %s\n", av_err2str(ret));
    exit(1);
  }

  frame = av_frame_alloc();
  if (!frame) {
    fprintf(stderr, "Could not allocate video frame\n");
    exit(1);
  }
  frame->format = encoder_context->pix_fmt;
  frame->width = encoder_context->width;
  frame->height = encoder_context->height;

  ret = av_frame_get_buffer(frame, 0);
  if (ret < 0) {
    fprintf(stderr, "Could not allocate the video frame data\n");
    exit(1);
  }

  std::vector<AVPacket *> encoded_pkts;

  std::vector<std::vector<uint8_t>> encoded_packet_data;
  /*

  Main Image Encoding loop
  */
  int frame_idx = 0;
  std::vector<cv::Mat> original_bgr_images;
  for (const auto &path : image_paths) {
    cv::Mat img = imread(path, cv::IMREAD_COLOR);
    if (img.size().width != encoder_context->width ||
        img.size().height != encoder_context->height) {
      std::cout << "Incorrect encoder image size" << std::endl;
    }
    if (img.empty()) {
      std::cout << "Could not read the image: " << path << std::endl;
      return 1;
    }

    ret = av_frame_make_writable(frame);
    if (ret < 0) exit(1);

    // // Test gray image
    // img = cv::Mat(img.size().width, img.size().height, CV_8UC3,
    //               cv::Scalar(127, 127, 127));

    cv::Mat img_yuv420p;
    cv::cvtColor(img, img_yuv420p, cv::COLOR_BGR2YUV_I420);

    // cv::Mat image_bgr;
    // cv::cvtColor(img_yuv420p, image_bgr, cv::COLOR_YUV2BGR_NV12, 3);
    // cv::imshow("test_image", image_bgr);
    // cv::waitKey(0);

    // Copy frame data from yuv channel
    int y_channel_size = img.size().width * img.size().height;
    int uv_channel_size = img.size().width * img.size().height / 4;
    original_bgr_images.push_back(img);
    std::memcpy(frame->data[0], img_yuv420p.data, y_channel_size);
    std::memcpy(frame->data[1], img_yuv420p.data + y_channel_size,
                uv_channel_size);
    std::memcpy(frame->data[2],
                img_yuv420p.data + y_channel_size + uv_channel_size,
                uv_channel_size);

    // av_image_fill_arrays(frame->data, frame->linesize, img_yuv420p.data,
    //                      static_cast<AVPixelFormat>(frame->format),
    //                      img.size().width, img.size().height, 32);

    // timestamp = index * timebase
    frame->pts = frame_idx;
    frame_idx++;

    // encode
    encode(encoder_context, frame, pkt);
    encoded_pkts.push_back(av_packet_clone(pkt));
    std::vector<uint8_t> pkt_data;
    pkt_data.resize(pkt->size);
    std::cout << "Pkt size " << pkt->size << std::endl;
    std::memcpy(pkt_data.data(), pkt->data, pkt->size);
    encoded_packet_data.push_back(pkt_data);
    av_packet_unref(pkt);
  }
  encode(encoder_context, NULL, pkt);

  int total_size = 0;
  for (const auto &pkt_data : encoded_packet_data) {
    total_size += pkt_data.size();
  }

  std::cout << "Number of Encoded packets:" << encoded_pkts.size()
            << " Total size = " << total_size << std::endl;

  /*
  Free Encoding resources
  */
  avcodec_free_context(&encoder_context);
  av_frame_free(&frame);
  av_packet_free(&pkt);

  /*
   * Decoding
   *
   *
   *
   *
   */

  const AVCodec *decoder;
  AVCodecParserContext *parser;
  AVCodecContext *decoder_context = NULL;
  AVFrame *decoded_frame;
  AVPacket *decode_pkt;

  int eof;
  decode_pkt = av_packet_alloc();
  if (!decode_pkt) exit(1);
  decoder = avcodec_find_decoder_by_name("h264");
  if (!decoder) {
    fprintf(stderr, "Decoder not found\n");
    exit(1);
  }
  parser = av_parser_init(decoder->id);
  if (!parser) {
    fprintf(stderr, "parser not found\n");
    exit(1);
  }

  // Set the PARSER_FLAG_COMPLETE_FRAMES to get the parsed data before the next
  // frame arrives. The packet is known to be completed.
  parser->flags |= PARSER_FLAG_COMPLETE_FRAMES;
  decoder_context = avcodec_alloc_context3(decoder);
  if (!decoder_context) {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }

  // Add Low Delay Flag for low latency application.
  // https://github.com/FFmpeg/FFmpeg/blob/eeb280f3518a8d7fc2e45f06ac7748f42d8a0000/libavcodec/cuviddec.c#L988
  decoder_context->flags |= AV_CODEC_FLAG_LOW_DELAY;
  // decoder_context->time_base = (AVRational){1, 10};

  /* open it */
  if (avcodec_open2(decoder_context, decoder, NULL) < 0) {
    fprintf(stderr, "Could not open codec\n");
    exit(1);
  }

  decoded_frame = av_frame_alloc();
  if (!decoded_frame) {
    fprintf(stderr, "Could not allocate video frame\n");
    exit(1);
  }

  // /*
  // Use Saved Packet
  // */
  // int pkt_idx = 0;
  // for (auto &pkt : encoded_pkts) {
  //   decode(decoder_context, decoded_frame, pkt);
  //   std::cout << "frame type " << decoded_frame->pict_type << std::endl;
  //   cv::Mat image(decoded_frame->height * 3 / 2, decoded_frame->width,
  //   CV_8UC1);

  //   int y_channel_size = decoded_frame->width * decoded_frame->height;
  //   int uv_channel_size = decoded_frame->width * decoded_frame->height / 4;
  //   // std::memcpy(image.data, decoded_frame->data[0], y_channel_size);
  //   // std::memcpy(image.data + y_channel_size, decoded_frame->data[1],
  //   //             uv_channel_size);
  //   // std::memcpy(image.data + y_channel_size + uv_channel_size,
  //   //             decoded_frame->data[2], uv_channel_size);

  //   int image_data_size = y_channel_size + 2 * uv_channel_size;
  //   av_image_copy_to_buffer(image.data, image_data_size, decoded_frame->data,
  //                           decoded_frame->linesize,
  //                           static_cast<AVPixelFormat>(decoded_frame->format),
  //                           decoded_frame->width, decoded_frame->height, 32);

  //   cv::Mat image_bgr(decoded_frame->height, decoded_frame->width, CV_8UC3);
  //   cv::cvtColor(image, image_bgr, cv::COLOR_YUV2BGR_I420, 3);
  //   cv::imshow("test_image", image_bgr);
  //   cv::waitKey(0);

  //   av_packet_free(&pkt);
  // }
  // decode(decoder_context, decoded_frame, NULL);

  /*
  Use packet data vector
  */
  std::vector<cv::Mat> decoded_frames;
  int pkt_idx = 0;
  for (auto &pkt_data : encoded_packet_data) {
    std::cout << "Parsing packet (size= " << pkt_data.size() << std::endl;

    ret = av_parser_parse2(parser, decoder_context, &decode_pkt->data,
                           &decode_pkt->size, pkt_data.data(), pkt_data.size(),
                           AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);

    std::cout << "Parse result 0 " << av_err2str(ret) << std::endl;

    if (decode_pkt->size) {
      std::cout << "Parsed a packet" << std::endl;
    }
    if (ret < 0) {
      fprintf(stderr, "Error while parsing\n");
      exit(1);
    }

    // if (decode_pkt->size) decode(decoder_context, decoded_frame,
    // decode_pkt);

    if (decode_pkt->size) {
      decode(decoder_context, decoded_frame, decode_pkt);

      cv::Mat image(decoded_frame->height * 3 / 2, decoded_frame->width,
                    CV_8UC1);
      int y_channel_size = decoded_frame->width * decoded_frame->height;
      int uv_channel_size = decoded_frame->width * decoded_frame->height / 4;

      int image_data_size = y_channel_size + 2 * uv_channel_size;
      av_image_copy_to_buffer(image.data, image_data_size, decoded_frame->data,
                              decoded_frame->linesize,
                              static_cast<AVPixelFormat>(decoded_frame->format),
                              decoded_frame->width, decoded_frame->height, 32);

      cv::Mat image_bgr(decoded_frame->height, decoded_frame->width, CV_8UC3);
      cv::cvtColor(image, image_bgr, cv::COLOR_YUV2BGR_I420, 3);
      // cv::imshow("test_image", image_bgr);
      // cv::waitKey(0);
      auto mse = cv::quality::QualityMSE::compute(original_bgr_images[pkt_idx],
                                                  image_bgr, cv::noArray());
      auto ssim = cv::quality::QualitySSIM::compute(
          original_bgr_images[pkt_idx], image_bgr, cv::noArray());
      auto psnr = cv::quality::QualityPSNR::compute(
          original_bgr_images[pkt_idx], image_bgr, cv::noArray());

      std::cout << "Image quality scores MSE = " << mse << " .SSIM = " << ssim
                << " .PSNR = " << psnr << std::endl;
      if (save_output) {
        auto output_path = output_dir / image_paths[pkt_idx].filename();
        cv::imwrite(output_path, image_bgr);
        std::cout << "Saved to output_path << " << output_path << std::endl;
      }
    } else {
      std::cout << "Missing frames for decoding " << av_err2str(ret)
                << std::endl;
      break;
    }
    pkt_idx++;
  }

  av_parser_close(parser);
  avcodec_free_context(&decoder_context);
  av_frame_free(&decoded_frame);
  av_packet_free(&decode_pkt);

  return 0;
}