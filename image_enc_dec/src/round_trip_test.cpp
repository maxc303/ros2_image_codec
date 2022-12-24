
// Need to warp ffmpeg headers into extern "C"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}
#include <boost/program_options.hpp>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
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

namespace po = boost::program_options;
int main(int argc, char **argv) {
  // Parse test infomration
  po::options_description desc(
      "FFmpeg API encoding and decoding round trip test");
  desc.add_options()("help", "produce help message")  //
      ("image_dir", po::value<std::string>(), "input images dir");

  po::variables_map boost_args;
  po::store(po::parse_command_line(argc, argv, desc), boost_args);
  po::notify(boost_args);

  if (boost_args.count("help") != 0 || boost_args.count("image_dir") == 0) {
    std::cout << desc << std::endl;
    return 1;
  }

  std::filesystem::path image_dir = boost_args["image_dir"].as<std::string>();
  std::vector<std::filesystem::path> image_paths;

  for (const auto &entry : std::filesystem::directory_iterator(image_dir)) {
    image_paths.push_back(entry.path());
  }
  std::sort(image_paths.begin(), image_paths.end());
  for (const auto &path : image_paths) {
    std::cout << path << std::endl;
  }

  /**
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
  encoder_context->bit_rate = 400000;
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
  encoder_context->gop_size = 10;
  encoder_context->max_b_frames = 1;
  encoder_context->pix_fmt = AV_PIX_FMT_BGR0;

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

  for (const auto &path : image_paths) {
    cv::Mat img = imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
      std::cout << "Could not read the image: " << path << std::endl;
      return 1;
    }
  }

  /*
  Free resources
  */
  avcodec_free_context(&encoder_context);
  av_frame_free(&frame);
  av_packet_free(&pkt);

  return 0;
}